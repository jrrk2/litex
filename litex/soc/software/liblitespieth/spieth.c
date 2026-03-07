// KSZ8851SNL SPI Ethernet driver for LiteX BIOS
// Provides UDP/IP/ARP/ICMP stack over KSZ8851 via spi_wb controller.
// RX dequeue logic adapted from proven eth_minimal_litex.c and Linux ks8851 driver.
// License: BSD

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#ifdef SPIETH_BASE

#include <stdio.h>
#include <string.h>
#include <system.h>

#include <liblitespieth/spieth.h>
#include <libliteeth/inet.h>

//#define SPIETH_DEBUG

// ============================================================================
// Layer 1: SPI controller access (spi_wb at SPIETH_BASE)
// ============================================================================

#define SPI_REG(off)    (*(volatile uint32_t *)(SPIETH_BASE + (off)))
#define SPI_TX_BUF      ((volatile uint32_t *)(SPIETH_BASE + 0x1000))
#define SPI_RX_BUF      ((volatile uint32_t *)(SPIETH_BASE + 0x1800))

// Register offsets (byte addresses)
#define SPI_INTR_STATE  0x00
#define SPI_INTR_ENABLE 0x04
#define SPI_CFG         0x0C
#define SPI_CONTROL     0x10
#define SPI_STATUS      0x14
#define SPI_START       0x18
#define SPI_CS          0x28

#define SPI_STATUS_IDLE (1 << 26)

static void spi_wait_idle(void)
{
	int timeout = 100000;
	while (!(SPI_REG(SPI_STATUS) & SPI_STATUS_IDLE)) {
		if (--timeout <= 0) {
			printf("SPI: idle timeout\n");
			return;
		}
	}
}

static void spi_configure(void)
{
	// Software reset of SPI controller (in case BIOS left it mid-transfer)
	SPI_REG(SPI_CONTROL) = (1u << 31);

	// MSB-first, CPOL=0, CPHA=0, half_clk_period=1 (25MHz SPI @ 50MHz sys)
	SPI_REG(SPI_CFG) = (1 << 29) | 1;
	SPI_REG(SPI_CS) = 1;  // CS de-asserted (active-low)
	spi_wait_idle();

	// Pulse CS a few times to reset KSZ8851 SPI state machine
	int i;
	for (i = 0; i < 5; i++) {
		SPI_REG(SPI_CS) = 0;
		busy_wait(1);
		SPI_REG(SPI_CS) = 1;
		busy_wait(1);
	}
}

// Write bytes to TX BRAM (word-aligned 32-bit writes)
static void tx_bram_write(const uint8_t *data, uint32_t len)
{
	uint32_t i;
	for (i = 0; i + 3 < len; i += 4) {
		SPI_TX_BUF[i/4] = data[i] | ((uint32_t)data[i+1] << 8) |
			((uint32_t)data[i+2] << 16) | ((uint32_t)data[i+3] << 24);
	}
	if (i < len) {
		uint32_t val = 0;
		uint32_t j;
		for (j = 0; j < len - i; j++)
			val |= (uint32_t)data[i+j] << (j*8);
		SPI_TX_BUF[i/4] = val;
	}
}

// Read bytes from RX BRAM
static void rx_bram_read(uint8_t *data, uint32_t offset, uint32_t len)
{
	uint32_t i = 0;
	uint32_t pos = offset;
	// Handle leading unaligned bytes
	while (i < len && (pos & 3)) {
		uint32_t word = SPI_RX_BUF[pos / 4];
		data[i++] = (word >> ((pos % 4) * 8)) & 0xFF;
		pos++;
	}
	// Handle aligned words
	while (i + 3 < len) {
		uint32_t word = SPI_RX_BUF[pos / 4];
		data[i]   = word & 0xFF;
		data[i+1] = (word >> 8) & 0xFF;
		data[i+2] = (word >> 16) & 0xFF;
		data[i+3] = (word >> 24) & 0xFF;
		i += 4;
		pos += 4;
	}
	// Handle trailing bytes
	while (i < len) {
		uint32_t word = SPI_RX_BUF[pos / 4];
		data[i++] = (word >> ((pos % 4) * 8)) & 0xFF;
		pos++;
	}
}

// Write bytes to TX BRAM at an arbitrary byte offset
static void tx_bram_write_at(const uint8_t *data, uint32_t offset, uint32_t len)
{
	uint32_t i = 0;
	uint32_t pos = offset;

	// Handle leading unaligned bytes (merge with existing word)
	if (pos & 3) {
		uint32_t word_idx = pos / 4;
		uint32_t val = SPI_TX_BUF[word_idx];
		while (i < len && (pos & 3)) {
			uint32_t shift = (pos & 3) * 8;
			val = (val & ~((uint32_t)0xFF << shift)) | ((uint32_t)data[i] << shift);
			i++; pos++;
		}
		SPI_TX_BUF[word_idx] = val;
	}

	// Handle aligned words
	while (i + 3 < len) {
		SPI_TX_BUF[pos / 4] = data[i] | ((uint32_t)data[i+1] << 8) |
			((uint32_t)data[i+2] << 16) | ((uint32_t)data[i+3] << 24);
		i += 4; pos += 4;
	}

	// Handle trailing bytes
	if (i < len) {
		uint32_t val = 0;
		uint32_t j;
		for (j = 0; i + j < len; j++)
			val |= (uint32_t)data[i+j] << (j * 8);
		SPI_TX_BUF[pos / 4] = val;
	}
}

// ============================================================================
// Layer 2: KSZ8851 SPI register and FIFO access
// ============================================================================

// KSZ8851 register map
#define KSZ_MARL    0x10
#define KSZ_MARM    0x12
#define KSZ_MARH    0x14
#define KSZ_GRR     0x26
#define KSZ_TXCR    0x70
#define KSZ_TXMIR   0x78
#define KSZ_TXQCR   0x80
#define KSZ_RXQCR   0x82
#define KSZ_TXFDPR  0x84
#define KSZ_RXFDPR  0x86
#define KSZ_RXCR1   0x74
#define KSZ_RXCR2   0x76
#define KSZ_RXFHSR  0x7C
#define KSZ_RXFHBCR 0x7E
#define KSZ_IER     0x90
#define KSZ_ISR     0x92
#define KSZ_RXFCTR  0x9C
#define KSZ_FCLWR   0xB0
#define KSZ_FCHWR   0xB2
#define KSZ_CIDER   0xC0
#define KSZ_P1MBCR  0xE4
#define KSZ_P1MBSR  0xE6
#define KSZ_P1CR    0xF6
#define KSZ_P1SR    0xF8

// RXFHSR bits
#define RX_VALID            (1 << 15)
#define RX_CRC_ERROR        (1 << 0)
#define RX_RUNT_FRAME       (1 << 1)
#define RX_FRAME_TOO_LONG   (1 << 2)
#define RX_MII_ERROR        (1 << 4)
#define RX_ERROR_MASK       (RX_CRC_ERROR | RX_RUNT_FRAME | RX_FRAME_TOO_LONG | RX_MII_ERROR)

// RXQCR bits (matching Linux ks8851 driver naming)
#define RXQCR_RRXEF         (1 << 0)  // Release RX frame (dequeue)
#define RXQCR_START_DMA     (1 << 3)  // Start DMA access
#define RXQCR_RXFCTE        (1 << 5)  // Frame count threshold enable

// TXQCR bits
#define TXQCR_MANUAL_ENQUEUE (1 << 0)

// ISR bits
#define IRQ_RXPSIS          (1 << 8)   // RX process stopped
#define IRQ_RXI             (1 << 13)  // RX interrupt

// Cached RXQCR base value (matching Linux driver's rc_rxqcr)
static uint16_t rc_rxqcr = RXQCR_RXFCTE;

static uint16_t ksz_reg_read(uint8_t reg)
{
	uint8_t be = (reg & 0x2) == 0 ? 0x03 : 0x0C;
	uint8_t cmd[4];
	cmd[0] = (0x00 << 6) | (be << 2) | (reg >> 6);
	cmd[1] = (reg << 2) & 0xF0;
	cmd[2] = 0;
	cmd[3] = 0;

	tx_bram_write(cmd, 4);
	SPI_REG(SPI_CS) = 0;
	SPI_REG(SPI_START) = 4;
	spi_wait_idle();
	uint32_t rx = SPI_RX_BUF[0];
	SPI_REG(SPI_CS) = 1;

	return (rx >> 16) & 0xFFFF;
}

static void ksz_reg_write(uint8_t reg, uint16_t val)
{
	uint8_t be = (reg & 0x2) == 0 ? 0x03 : 0x0C;
	uint8_t cmd[4];
	cmd[0] = (0x01 << 6) | (be << 2) | (reg >> 6);
	cmd[1] = (reg << 2) & 0xF0;
	cmd[2] = val & 0xFF;
	cmd[3] = (val >> 8) & 0xFF;

	tx_bram_write(cmd, 4);
	SPI_REG(SPI_CS) = 0;
	SPI_REG(SPI_START) = 4;
	spi_wait_idle();
	SPI_REG(SPI_CS) = 1;
}

static void ksz_reg_set(uint8_t reg, uint16_t mask)
{
	ksz_reg_write(reg, ksz_reg_read(reg) | mask);
}

// Send a frame via KSZ8851 TX FIFO (using rc_rxqcr DMA pattern)
static int ksz_send_frame(const uint8_t *frame, uint16_t len)
{
	// Wait for TX FIFO space
	int timeout = 100000;
	while (timeout-- > 0) {
		uint16_t txmir = ksz_reg_read(KSZ_TXMIR) & 0x0FFF;
		if (txmir >= len + 4)
			break;
	}
	if (timeout <= 0) {
		printf("KSZ8851: TX FIFO full\n");
		return -1;
	}

	ksz_reg_write(KSZ_TXFDPR, 0x4000);

	// Start TX DMA access
	ksz_reg_write(KSZ_RXQCR, rc_rxqcr | RXQCR_START_DMA);

	// Build SPI FIFO write packet in TX BRAM:
	// [cmd(1)] [header(4)] [frame(len)] [padding to 4-byte boundary]
	uint32_t pad = (-(uint32_t)len) & 0x3;
	uint32_t total = 1 + 4 + len + pad;

	// Write command byte + TX header (5 bytes)
	uint8_t hdr[5];
	hdr[0] = 0xC0;  // FIFO write command (0b11 << 6)
	hdr[1] = 0x00;  // TX control word low
	hdr[2] = 0x80;  // TX control word high (TXIC=1)
	hdr[3] = len & 0xFF;
	hdr[4] = len >> 8;
	tx_bram_write(hdr, 5);

	// Write frame data at byte offset 5
	tx_bram_write_at(frame, 5, len);

	// Zero padding (if any)
	if (pad > 0) {
		uint8_t zeros[4] = {0, 0, 0, 0};
		tx_bram_write_at(zeros, 5 + len, pad);
	}

	// SPI transfer
	SPI_REG(SPI_CS) = 0;
	SPI_REG(SPI_START) = total;
	spi_wait_idle();
	SPI_REG(SPI_CS) = 1;

	// End TX DMA access (no RRXEF - don't release RX frame)
	ksz_reg_write(KSZ_RXQCR, rc_rxqcr);

	// Enqueue TX frame
	ksz_reg_set(KSZ_TXQCR, TXQCR_MANUAL_ENQUEUE);

	return 0;
}

void spieth_init(void)
{
	printf("SPI Ethernet init...\n");

	// Configure SPI controller (includes SW reset + CS pulses)
	spi_configure();

	// Software reset KSZ8851
	ksz_reg_write(KSZ_GRR, 0x0001);
	busy_wait(50);
	ksz_reg_write(KSZ_GRR, 0x0000);
	busy_wait(50);

	// Verify chip ID
	uint16_t cider = ksz_reg_read(KSZ_CIDER);
	printf("KSZ8851: Chip ID 0x%04x\n", cider);
	if ((cider & 0xFFF0) != 0x8870) {
		printf("KSZ8851: Unexpected chip ID, retrying...\n");
		// Retry with full SPI reset
		spi_configure();
		ksz_reg_write(KSZ_GRR, 0x0001);
		busy_wait(50);
		ksz_reg_write(KSZ_GRR, 0x0000);
		busy_wait(50);
		cider = ksz_reg_read(KSZ_CIDER);
		printf("KSZ8851: Retry Chip ID 0x%04x\n", cider);
		if ((cider & 0xFFF0) != 0x8870) {
			printf("KSZ8851: FATAL bad chip ID!\n");
			return;
		}
	}

	// TX init
	ksz_reg_write(KSZ_TXFDPR, 0x4000);
	ksz_reg_write(KSZ_TXCR, 0x00EE);

	// RX init
	ksz_reg_write(KSZ_RXFDPR, 0x4000);
	ksz_reg_write(KSZ_RXFCTR, 0x0001);
	ksz_reg_write(KSZ_RXCR1, 0x7CE0);  // unicast + broadcast + multicast
	ksz_reg_write(KSZ_RXCR2, 0x009C);
	// Use RXFCTE only — NO AUTO_DEQUEUE (we release frames explicitly with RRXEF)
	ksz_reg_write(KSZ_RXQCR, rc_rxqcr);

	// Restart auto-negotiation
	ksz_reg_set(KSZ_P1CR, 1 << 13);

	// Configure flow control watermarks
	ksz_reg_write(KSZ_FCLWR, 0x0600);
	ksz_reg_write(KSZ_FCHWR, 0x0400);

	// Clear and enable interrupts (polling mode — enable RXI, TXI, RXPSIS)
	ksz_reg_write(KSZ_ISR, 0xFFFF);
	ksz_reg_write(KSZ_IER, 0xE000);

	// Enable TX and RX
	ksz_reg_set(KSZ_TXCR, 1 << 0);
	ksz_reg_set(KSZ_RXCR1, 1 << 0);

	// Wait for PHY link
	printf("KSZ8851: Waiting for link...\n");
	int timeout = 5000;
	while (timeout-- > 0) {
		uint16_t p1sr = ksz_reg_read(KSZ_P1SR);
		if (p1sr & (1 << 5)) {  // Link good
			printf("KSZ8851: Link up\n");
			return;
		}
		busy_wait(1);
	}
	printf("KSZ8851: Link timeout (may work later)\n");
}

void eth_init(void)
{
	spieth_init();
}

// ============================================================================
// Layer 3: UDP/IP/ARP/ICMP stack (adapted from libliteeth/udp.c)
// ============================================================================

// KSZ8851 handles preamble and CRC, so frames start at MAC header

#define IPTOINT(a, b, c, d) ((a << 24)|(b << 16)|(c << 8)|d)

#define ETHERTYPE_ARP 0x0806
#define ETHERTYPE_IP  0x0800

#define ARP_HWTYPE_ETHERNET 0x0001
#define ARP_PROTO_IP        0x0800
#define ARP_PACKET_LENGTH   60

#define ARP_OPCODE_REQUEST  0x0001
#define ARP_OPCODE_REPLY    0x0002

#define IP_IPV4          0x45
#define IP_DONT_FRAGMENT 0x4000
#define IP_TTL           64
#define IP_PROTO_UDP     0x11
#define IP_PROTO_ICMP    0x01

#define ICMP_ECHO_REPLY  0x00
#define ICMP_ECHO        0x08

#define FRAME_BUF_SIZE   1536

struct ethernet_header {
	uint8_t destmac[6];
	uint8_t srcmac[6];
	uint16_t ethertype;
} __attribute__((packed));

struct arp_frame {
	uint16_t hwtype;
	uint16_t proto;
	uint8_t hwsize;
	uint8_t protosize;
	uint16_t opcode;
	uint8_t sender_mac[6];
	uint32_t sender_ip;
	uint8_t target_mac[6];
	uint32_t target_ip;
	uint8_t padding[18];
} __attribute__((packed));

struct ip_header {
	uint8_t version;
	uint8_t diff_services;
	uint16_t total_length;
	uint16_t identification;
	uint16_t fragment_offset;
	uint8_t ttl;
	uint8_t proto;
	uint16_t checksum;
	uint32_t src_ip;
	uint32_t dst_ip;
} __attribute__((packed));

struct udp_header {
	uint16_t src_port;
	uint16_t dst_port;
	uint16_t length;
	uint16_t checksum;
} __attribute__((packed));

struct udp_frame {
	struct ip_header ip;
	struct udp_header udp;
	char payload[];
} __attribute__((packed));

struct icmp_header {
	unsigned char type;
	unsigned char code;
	unsigned short checksum;
	unsigned short identifier;
	unsigned short sequence_number;
} __attribute__((packed));

struct icmp_frame {
	struct ip_header ip;
	struct icmp_header icmp;
	char payload[];
} __attribute__((packed));

struct ethernet_frame {
	struct ethernet_header eth_header;
	union {
		struct arp_frame arp;
		struct udp_frame udp;
		struct icmp_frame icmp;
	} contents;
} __attribute__((packed));

typedef union {
	struct ethernet_frame frame;
	uint8_t raw[FRAME_BUF_SIZE];
} ethernet_buffer;

static ethernet_buffer txbuf_storage;
static ethernet_buffer rxbuf_storage;
static ethernet_buffer *txbuffer = &txbuf_storage;
static ethernet_buffer *rxbuffer = &rxbuf_storage;
static uint32_t txlen;
static uint32_t rxlen;

static uint8_t my_mac[6];
static uint32_t my_ip;

// ARP cache - one entry
static uint8_t cached_mac[6];
static uint32_t cached_ip;

static udp_callback rx_callback;

static void fill_eth_header(struct ethernet_header *h,
	const uint8_t *destmac, const uint8_t *srcmac, uint16_t ethertype)
{
	int i;
	for (i = 0; i < 6; i++)
		h->destmac[i] = destmac[i];
	for (i = 0; i < 6; i++)
		h->srcmac[i] = srcmac[i];
	h->ethertype = htons(ethertype);
}

static void send_packet(void)
{
#ifdef SPIETH_DEBUG
	printf("TX %ld bytes\n", txlen);
#endif
	ksz_send_frame(txbuffer->raw, txlen);
}

static uint16_t ip_checksum(uint32_t r, void *buffer, uint32_t length, int complete)
{
	uint8_t *ptr;
	uint32_t i;

	ptr = (uint8_t *)buffer;
	length >>= 1;

	for (i = 0; i < length; i++)
		r += ((uint32_t)(ptr[2*i]) << 8) | (uint32_t)(ptr[2*i+1]);

	while (r >> 16)
		r = (r & 0xffff) + (r >> 16);

	if (complete) {
		r = ~r;
		r &= 0xffff;
		if (r == 0) r = 0xffff;
	}
	return r;
}

void udp_set_ip(uint32_t ip)
{
	my_ip = ip;
}

uint32_t udp_get_ip(void)
{
	return my_ip;
}

void udp_set_mac(const uint8_t *macaddr)
{
	int i;
	for (i = 0; i < 6; i++)
		my_mac[i] = macaddr[i];

	// Also write MAC to KSZ8851
	ksz_reg_write(KSZ_MARH, (macaddr[0] << 8) | macaddr[1]);
	ksz_reg_write(KSZ_MARM, (macaddr[2] << 8) | macaddr[3]);
	ksz_reg_write(KSZ_MARL, (macaddr[4] << 8) | macaddr[5]);
}

void *udp_get_tx_buffer(void)
{
	return txbuffer->frame.contents.udp.payload;
}

static const uint8_t broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static void process_arp(void)
{
	const struct arp_frame *rx_arp = &rxbuffer->frame.contents.arp;
	struct arp_frame *tx_arp = &txbuffer->frame.contents.arp;

	if (rxlen < ARP_PACKET_LENGTH) return;
	if (ntohs(rx_arp->hwtype) != ARP_HWTYPE_ETHERNET) return;
	if (ntohs(rx_arp->proto) != ARP_PROTO_IP) return;
	if (rx_arp->hwsize != 6) return;
	if (rx_arp->protosize != 4) return;

	if (ntohs(rx_arp->opcode) == ARP_OPCODE_REPLY) {
		if (ntohl(rx_arp->sender_ip) == cached_ip) {
			int i;
			for (i = 0; i < 6; i++)
				cached_mac[i] = rx_arp->sender_mac[i];
		}
		return;
	}
	if (ntohs(rx_arp->opcode) == ARP_OPCODE_REQUEST) {
		if (ntohl(rx_arp->target_ip) == my_ip) {
			int i;
			fill_eth_header(&txbuffer->frame.eth_header,
				rx_arp->sender_mac, my_mac, ETHERTYPE_ARP);
			txlen = ARP_PACKET_LENGTH;
			tx_arp->hwtype = htons(ARP_HWTYPE_ETHERNET);
			tx_arp->proto = htons(ARP_PROTO_IP);
			tx_arp->hwsize = 6;
			tx_arp->protosize = 4;
			tx_arp->opcode = htons(ARP_OPCODE_REPLY);
			tx_arp->sender_ip = htonl(my_ip);
			for (i = 0; i < (int)sizeof(tx_arp->padding); i++)
				tx_arp->padding[i] = 0;
			for (i = 0; i < 6; i++)
				tx_arp->sender_mac[i] = my_mac[i];
			tx_arp->target_ip = htonl(ntohl(rx_arp->sender_ip));
			for (i = 0; i < 6; i++)
				tx_arp->target_mac[i] = rx_arp->sender_mac[i];
			send_packet();
		}
		return;
	}
}

int udp_arp_resolve(uint32_t ip)
{
	struct arp_frame *arp;
	int i;
	int tries;
	int timeout;

	if (cached_ip == ip) {
		for (i = 0; i < 6; i++)
			if (cached_mac[i]) return 1;
	}
	cached_ip = ip;
	for (i = 0; i < 6; i++)
		cached_mac[i] = 0;

	for (tries = 0; tries < 8; tries++) {
		fill_eth_header(&txbuffer->frame.eth_header,
			broadcast, my_mac, ETHERTYPE_ARP);
		txlen = ARP_PACKET_LENGTH;
		arp = &txbuffer->frame.contents.arp;
		arp->hwtype = htons(ARP_HWTYPE_ETHERNET);
		arp->proto = htons(ARP_PROTO_IP);
		arp->hwsize = 6;
		arp->protosize = 4;
		arp->opcode = htons(ARP_OPCODE_REQUEST);
		arp->sender_ip = htonl(my_ip);
		for (i = 0; i < (int)sizeof(arp->padding); i++)
			arp->padding[i] = 0;
		for (i = 0; i < 6; i++)
			arp->sender_mac[i] = my_mac[i];
		arp->target_ip = htonl(ip);
		for (i = 0; i < 6; i++)
			arp->target_mac[i] = 0;

		send_packet();

		for (timeout = 0; timeout < 100000; timeout++) {
			udp_service();
			for (i = 0; i < 6; i++)
				if (cached_mac[i]) return 1;
		}
	}

	return 0;
}

struct pseudo_header {
	uint32_t src_ip;
	uint32_t dst_ip;
	uint8_t zero;
	uint8_t proto;
	uint16_t length;
} __attribute__((packed));

int udp_send(uint16_t src_port, uint16_t dst_port, uint32_t length)
{
	struct pseudo_header h;
	uint32_t r;

	if ((cached_mac[0] == 0) && (cached_mac[1] == 0) && (cached_mac[2] == 0)
		&& (cached_mac[3] == 0) && (cached_mac[4] == 0) && (cached_mac[5] == 0))
		return 0;

	txlen = length + sizeof(struct ethernet_header) + sizeof(struct udp_frame);
	if (txlen < ARP_PACKET_LENGTH) txlen = ARP_PACKET_LENGTH;

	fill_eth_header(&txbuffer->frame.eth_header,
		cached_mac, my_mac, ETHERTYPE_IP);

	txbuffer->frame.contents.udp.ip.version = IP_IPV4;
	txbuffer->frame.contents.udp.ip.diff_services = 0;
	txbuffer->frame.contents.udp.ip.total_length = htons(length + sizeof(struct udp_frame));
	txbuffer->frame.contents.udp.ip.identification = htons(0);
	txbuffer->frame.contents.udp.ip.fragment_offset = htons(IP_DONT_FRAGMENT);
	txbuffer->frame.contents.udp.ip.ttl = IP_TTL;
	h.proto = txbuffer->frame.contents.udp.ip.proto = IP_PROTO_UDP;
	txbuffer->frame.contents.udp.ip.checksum = 0;
	h.src_ip = txbuffer->frame.contents.udp.ip.src_ip = htonl(my_ip);
	h.dst_ip = txbuffer->frame.contents.udp.ip.dst_ip = htonl(cached_ip);
	txbuffer->frame.contents.udp.ip.checksum = htons(ip_checksum(0,
		&txbuffer->frame.contents.udp.ip, sizeof(struct ip_header), 1));

	txbuffer->frame.contents.udp.udp.src_port = htons(src_port);
	txbuffer->frame.contents.udp.udp.dst_port = htons(dst_port);
	h.length = txbuffer->frame.contents.udp.udp.length = htons(length + sizeof(struct udp_header));
	txbuffer->frame.contents.udp.udp.checksum = 0;

	h.zero = 0;
	r = ip_checksum(0, &h, sizeof(struct pseudo_header), 0);
	if (length & 1) {
		txbuffer->frame.contents.udp.payload[length] = 0;
		length++;
	}
	r = ip_checksum(r, &txbuffer->frame.contents.udp.udp,
		sizeof(struct udp_header)+length, 1);
	txbuffer->frame.contents.udp.udp.checksum = htons(r);

	send_packet();
	return 1;
}

static unsigned ping_seq_number = 0;
static uint64_t ping_ts_send = 0;

int send_ping(uint32_t ip, unsigned short payload_length)
{
	if (!udp_arp_resolve(ip)) {
		printf("ARP failed\n");
		return -1;
	}

	fill_eth_header(&txbuffer->frame.eth_header,
		cached_mac, my_mac, ETHERTYPE_IP);

	struct icmp_frame *tx_icmp = &txbuffer->frame.contents.icmp;

	tx_icmp->ip.version = IP_IPV4;
	tx_icmp->ip.diff_services = 0;
	tx_icmp->ip.total_length = htons(payload_length + sizeof(struct icmp_frame));
	tx_icmp->ip.identification = htons(0);
	tx_icmp->ip.fragment_offset = htons(IP_DONT_FRAGMENT);
	tx_icmp->ip.ttl = IP_TTL;
	tx_icmp->ip.proto = IP_PROTO_ICMP;
	tx_icmp->ip.checksum = 0;
	tx_icmp->ip.src_ip = htonl(my_ip);
	tx_icmp->ip.dst_ip = htonl(ip);
	tx_icmp->ip.checksum = htons(ip_checksum(0, &tx_icmp->ip,
		sizeof(struct ip_header), 1));

	tx_icmp->icmp.type = ICMP_ECHO;
	tx_icmp->icmp.code = 0;
	tx_icmp->icmp.identifier = 0xbe7c;
	tx_icmp->icmp.sequence_number = ++ping_seq_number;
	{
		unsigned i;
		for (i = 0; i < payload_length; i++)
			tx_icmp->payload[i] = i;
	}

	tx_icmp->icmp.checksum = 0;
	unsigned short r = ip_checksum(0, &tx_icmp->icmp,
		payload_length + sizeof(struct icmp_header), 1);
	tx_icmp->icmp.checksum = htons(r);

	txlen = payload_length + sizeof(struct ethernet_header) + sizeof(struct icmp_frame);
	send_packet();

	ping_ts_send = 1;
#ifdef CSR_TIMER0_UPTIME_CYCLES_ADDR
	timer0_uptime_latch_write(1);
	ping_ts_send = timer0_uptime_cycles_read();
#endif

	// Wait for reply
	unsigned timeout;
	for (timeout = 0; timeout < 10000; timeout++) {
		udp_service();
		if (ping_ts_send == 0)
			return 0;
	}

	return -2;
}

static void process_icmp(void)
{
	if (rxlen < (sizeof(struct ethernet_header) + sizeof(struct icmp_frame)))
		return;

	const struct icmp_frame *rx_icmp = &rxbuffer->frame.contents.icmp;
	struct icmp_frame *tx_icmp = &txbuffer->frame.contents.icmp;

	if (ntohs(rx_icmp->ip.total_length) < sizeof(struct icmp_frame))
		return;

	unsigned short length = ntohs(rx_icmp->ip.total_length) - sizeof(struct icmp_frame);

	if (rx_icmp->icmp.type == ICMP_ECHO) {
		fill_eth_header(&txbuffer->frame.eth_header,
			rxbuffer->frame.eth_header.srcmac, my_mac, ETHERTYPE_IP);

		tx_icmp->ip.version = IP_IPV4;
		tx_icmp->ip.diff_services = 0;
		tx_icmp->ip.total_length = htons(length + sizeof(struct icmp_frame));
		tx_icmp->ip.identification = htons(0);
		tx_icmp->ip.fragment_offset = htons(IP_DONT_FRAGMENT);
		tx_icmp->ip.ttl = IP_TTL;
		tx_icmp->ip.proto = IP_PROTO_ICMP;
		tx_icmp->ip.checksum = 0;
		tx_icmp->ip.src_ip = htonl(my_ip);
		tx_icmp->ip.dst_ip = rxbuffer->frame.contents.icmp.ip.src_ip;
		tx_icmp->ip.checksum = htons(ip_checksum(0, &tx_icmp->ip,
			sizeof(struct ip_header), 1));

		tx_icmp->icmp.type = ICMP_ECHO_REPLY;
		tx_icmp->icmp.code = 0;
		tx_icmp->icmp.identifier = rx_icmp->icmp.identifier;
		tx_icmp->icmp.sequence_number = rx_icmp->icmp.sequence_number;
		{
			unsigned i;
			for (i = 0; i < length; i++)
				tx_icmp->payload[i] = rx_icmp->payload[i];
		}

		tx_icmp->icmp.checksum = 0;
		unsigned short r = ip_checksum(0, &tx_icmp->icmp,
			length + sizeof(struct icmp_header), 1);
		tx_icmp->icmp.checksum = htons(r);

		txlen = length + sizeof(struct ethernet_header) + sizeof(struct icmp_frame);
		send_packet();
	} else if (rx_icmp->icmp.type == ICMP_ECHO_REPLY) {
		uint8_t *tmp = (uint8_t *)(&rx_icmp->ip.src_ip);
		printf("%d bytes from %d.%d.%d.%d: ", length, tmp[0], tmp[1], tmp[2], tmp[3]);

		if (rx_icmp->icmp.sequence_number != ping_seq_number) {
			printf("invalid seq %d\n", rx_icmp->icmp.sequence_number);
			return;
		}
		if (rx_icmp->icmp.identifier != 0xbe7c) {
			printf("invalid id %d\n", rx_icmp->icmp.identifier);
			return;
		}

		printf("icmp_seq=%d", rx_icmp->icmp.sequence_number);

#ifdef CSR_TIMER0_UPTIME_CYCLES_ADDR
		uint64_t ping_ts_receive = 0;
		timer0_uptime_latch_write(1);
		ping_ts_receive = timer0_uptime_cycles_read();
		int dt_us = ping_ts_receive - ping_ts_send;
		dt_us /= (CONFIG_CLOCK_FREQUENCY / 1000 / 1000);
		if (dt_us >= 10000)
			printf(" time=%d ms", dt_us / 1000);
		else
			printf(" time=%d us", dt_us);
#endif

		ping_ts_send = 0;
		printf("\n");
	}
}

static void process_udp(void)
{
	if (rxlen < (sizeof(struct ethernet_header) + sizeof(struct udp_frame))) return;
	struct udp_frame *udp_ip = &rxbuffer->frame.contents.udp;
	if (ntohs(udp_ip->ip.total_length) < sizeof(struct udp_frame)) return;
	if (udp_ip->ip.proto != IP_PROTO_UDP) return;
	if (ntohs(udp_ip->udp.length) < sizeof(struct udp_header)) return;
	// Accept packets for our IP, broadcasts, or anything when IP not yet assigned (DHCP)
	{
		uint32_t dst = ntohl(udp_ip->ip.dst_ip);
		if (dst != my_ip && dst != IPTOINT(255,255,255,255) && my_ip != 0)
			return;
	}

	if (rx_callback) {
		rx_callback(ntohl(udp_ip->ip.src_ip), ntohs(udp_ip->udp.src_port),
			ntohs(udp_ip->udp.dst_port),
			udp_ip->payload, ntohs(udp_ip->udp.length) - sizeof(struct udp_header));
	}
}

static void process_frame(void)
{
#ifdef SPIETH_DEBUG
	printf("RX %ld bytes ethertype=0x%04x\n", rxlen,
		ntohs(rxbuffer->frame.eth_header.ethertype));
#endif

	// Skip our own TX (loopback in promiscuous mode)
	if (memcmp(rxbuffer->frame.eth_header.srcmac, my_mac, 6) == 0)
		return;

	if (ntohs(rxbuffer->frame.eth_header.ethertype) == ETHERTYPE_ARP) {
		process_arp();
	} else if (ntohs(rxbuffer->frame.eth_header.ethertype) == ETHERTYPE_IP) {
		struct ip_header *hdr = &rxbuffer->frame.contents.udp.ip;
		if (hdr->version != IP_IPV4)
			return;
		if (ntohl(hdr->dst_ip) != my_ip)
			return;
		if (hdr->proto == IP_PROTO_UDP)
			process_udp();
		else if (hdr->proto == IP_PROTO_ICMP)
			process_icmp();
	}
}

void udp_set_callback(udp_callback callback)
{
	rx_callback = callback;
}

void udp_start(const uint8_t *macaddr, uint32_t ip)
{
	int i;
	udp_set_ip(ip);
	udp_set_mac(macaddr);

	cached_ip = 0;
	for (i = 0; i < 6; i++)
		cached_mac[i] = 0;

	memset(&txbuf_storage, 0, sizeof(txbuf_storage));
	memset(&rxbuf_storage, 0, sizeof(rxbuf_storage));
	rx_callback = (udp_callback)0;
}

// ISR-gated RX poll: only process frames when IRQ_RXI is set,
// handle RXPSIS, and use RRXEF for explicit frame release.
// This matches the proven logic from eth_minimal_litex.c and Linux ks8851 driver.
void udp_service(void)
{
	uint16_t isr = ksz_reg_read(KSZ_ISR);
	if (isr)
		ksz_reg_write(KSZ_ISR, isr);

	// Handle RX process stopped (re-enable RX)
	if (isr & IRQ_RXPSIS) {
		uint16_t rxcr1 = ksz_reg_read(KSZ_RXCR1);
		if (!(rxcr1 & (1 << 0)))
			ksz_reg_write(KSZ_RXCR1, rxcr1 | (1 << 0));
	}

	// Only process RX if RX interrupt is set
	if (!(isr & IRQ_RXI))
		return;

	uint16_t frames = ksz_reg_read(KSZ_RXFCTR) >> 8;
	if (frames == 0)
		return;

	uint16_t f;
	for (f = 0; f < frames; f++) {
		uint16_t status = ksz_reg_read(KSZ_RXFHSR);
		uint16_t len    = ksz_reg_read(KSZ_RXFHBCR) & 0xFFF;

		int valid = (status & RX_VALID) && !(status & RX_ERROR_MASK);

		if (!valid || len <= 4 || len > FRAME_BUF_SIZE) {
			// Release invalid frame without DMA (matching Linux/Ibex)
			ksz_reg_write(KSZ_RXQCR, rc_rxqcr | RXQCR_RRXEF);
			continue;
		}

		uint16_t frame_len = len - 4;  // Strip 4-byte CRC
		uint16_t rxalign = (len + 3) & ~3;

		// Set DMA read address
		ksz_reg_write(KSZ_RXFDPR, 0x4000);
		// Start DMA access
		ksz_reg_write(KSZ_RXQCR, rc_rxqcr | RXQCR_START_DMA);

		// SPI FIFO read: [0x80 cmd] [8 header/dummy] [rxalign data]
		uint32_t fifo_len = rxalign + 8;
		uint32_t total = 1 + fifo_len;

		// Fill TX BRAM with read command
		SPI_TX_BUF[0] = 0x80;  // FIFO read command byte
		uint32_t w;
		for (w = 1; w < (total + 3) / 4; w++)
			SPI_TX_BUF[w] = 0;

		SPI_REG(SPI_CS) = 0;
		SPI_REG(SPI_START) = total;
		spi_wait_idle();

		// Frame data starts at byte offset 9 (1 cmd + 8 header/dummy)
		rx_bram_read(rxbuffer->raw, 9, frame_len);

		SPI_REG(SPI_CS) = 1;

		// Release frame: end DMA + RRXEF
		ksz_reg_write(KSZ_RXQCR, rc_rxqcr | RXQCR_RRXEF);

		rxlen = frame_len;
		process_frame();
	}
}

// ============================================================================
// DHCP client
// ============================================================================

#define DHCP_SERVER_PORT 67
#define DHCP_CLIENT_PORT 68
#define DHCP_MAGIC_COOKIE 0x63825363

#define DHCP_DISCOVER 1
#define DHCP_OFFER    2
#define DHCP_REQUEST  3
#define DHCP_ACK      5

struct dhcp_packet {
	uint8_t op;
	uint8_t htype;
	uint8_t hlen;
	uint8_t hops;
	uint32_t xid;
	uint16_t secs;
	uint16_t flags;
	uint32_t ciaddr;
	uint32_t yiaddr;
	uint32_t siaddr;
	uint32_t giaddr;
	uint8_t chaddr[16];
	uint8_t sname[64];
	uint8_t file[128];
	uint32_t magic;
	uint8_t options[];
} __attribute__((packed));

static uint32_t dhcp_xid = 0x12345678;
static uint32_t dhcp_offered_ip;
static uint32_t dhcp_server_ip;

static void dhcp_rx_callback(uint32_t src_ip, uint16_t src_port,
    uint16_t dst_port, void *data, uint32_t length)
{
	if (dst_port != DHCP_CLIENT_PORT) return;
	if (length < sizeof(struct dhcp_packet)) return;

	struct dhcp_packet *pkt = (struct dhcp_packet *)data;
	if (pkt->op != 2) return;  // Not a reply
	if (pkt->xid != htonl(dhcp_xid)) return;

	dhcp_offered_ip = ntohl(pkt->yiaddr);
	dhcp_server_ip = ntohl(pkt->siaddr);
}

static void dhcp_send(uint8_t msg_type, uint32_t requested_ip)
{
	char *payload = udp_get_tx_buffer();
	memset(payload, 0, 300);

	struct dhcp_packet *pkt = (struct dhcp_packet *)payload;
	pkt->op = 1;     // BOOTREQUEST
	pkt->htype = 1;  // Ethernet
	pkt->hlen = 6;
	pkt->hops = 0;
	pkt->xid = htonl(dhcp_xid);
	pkt->secs = 0;
	pkt->flags = htons(0x8000);  // Broadcast
	memcpy(pkt->chaddr, my_mac, 6);
	pkt->magic = htonl(DHCP_MAGIC_COOKIE);

	// Options
	uint8_t *opt = pkt->options;
	// Message type
	*opt++ = 53; *opt++ = 1; *opt++ = msg_type;
	// Requested IP (for DHCP Request)
	if (msg_type == DHCP_REQUEST && requested_ip) {
		*opt++ = 50; *opt++ = 4;
		uint32_t rip = htonl(requested_ip);
		memcpy(opt, &rip, 4); opt += 4;
		// Server identifier
		*opt++ = 54; *opt++ = 4;
		uint32_t sip = htonl(dhcp_server_ip);
		memcpy(opt, &sip, 4); opt += 4;
	}
	// End
	*opt++ = 255;

	uint32_t pkt_len = opt - (uint8_t *)payload;
	if (pkt_len < 300) pkt_len = 300;  // Minimum DHCP size

	// Use broadcast for DHCP
	int i;
	for (i = 0; i < 6; i++)
		cached_mac[i] = 0xFF;
	cached_ip = IPTOINT(255, 255, 255, 255);

	// Temporarily set my_ip to 0 for DHCP Discover source
	uint32_t saved_ip = my_ip;
	my_ip = 0;
	udp_send(DHCP_CLIENT_PORT, DHCP_SERVER_PORT, pkt_len);
	my_ip = saved_ip;
}

int spieth_dhcp(void)
{
	udp_callback old_callback = rx_callback;
	uint32_t old_ip = my_ip;
	int timeout, tries;

	printf("DHCP: Discovering...\n");

	// Use 0.0.0.0 as source during discovery
	my_ip = 0;
	udp_set_callback(dhcp_rx_callback);

	for (tries = 0; tries < 4; tries++) {
		dhcp_offered_ip = 0;
		dhcp_server_ip = 0;

		// Send DHCP Discover
		dhcp_send(DHCP_DISCOVER, 0);

		// Wait for offer
		for (timeout = 0; timeout < 500000; timeout++) {
			udp_service();
			if (dhcp_offered_ip != 0)
				break;
		}

		if (dhcp_offered_ip != 0)
			break;
	}

	if (dhcp_offered_ip == 0) {
		printf("DHCP: No offer received\n");
		my_ip = old_ip;
		udp_set_callback(old_callback);
		return -1;
	}

	printf("DHCP: Offered %lu.%lu.%lu.%lu\n",
		(dhcp_offered_ip >> 24) & 0xFF, (dhcp_offered_ip >> 16) & 0xFF,
		(dhcp_offered_ip >> 8) & 0xFF, dhcp_offered_ip & 0xFF);

	// Send DHCP Request
	dhcp_send(DHCP_REQUEST, dhcp_offered_ip);

	// Wait for ACK
	dhcp_offered_ip = 0;
	for (timeout = 0; timeout < 500000; timeout++) {
		udp_service();
		if (dhcp_offered_ip != 0)
			break;
	}

	udp_set_callback(old_callback);

	if (dhcp_offered_ip != 0) {
		my_ip = dhcp_offered_ip;
		printf("DHCP: Assigned %lu.%lu.%lu.%lu\n",
			(my_ip >> 24) & 0xFF, (my_ip >> 16) & 0xFF,
			(my_ip >> 8) & 0xFF, my_ip & 0xFF);
		return 0;
	}

	printf("DHCP: No ACK received\n");
	my_ip = old_ip;
	return -1;
}

#endif /* SPIETH_BASE */
