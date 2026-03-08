// BIOS-integrated KSZ8851 Ethernet - DHCP + ARP + Ping + TFTP boot
// Adapted from test/eth_minimal_litex.c for LiteX BIOS ROM
// No lwIP, no interrupts, no timers - pure polling

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/soc.h>

#include "boot.h"

#ifdef SPIETH_BASE

// ---- Hardware addresses (from soc.h SPIETH_BASE) ----

#define SPI_REG(off)   (*(volatile uint32_t *)(SPIETH_BASE + (off)))
#define SPI_TX_BUF     ((volatile uint32_t *)(SPIETH_BASE + 0x1000))
#define SPI_RX_BUF     ((volatile uint32_t *)(SPIETH_BASE + 0x1800))

// SPI controller registers (spi_wb.sv)
#define SPI_CFG         0x0C
#define SPI_CONTROL     0x10
#define SPI_STATUS      0x14
#define SPI_START       0x18
#define SPI_CS          0x28
#define SPI_STATUS_IDLE (1 << 26)

// ---- KSZ8851 register defines ----

#define ETH_MARL    0x10
#define ETH_MARM    0x12
#define ETH_MARH    0x14
#define ETH_GRR     0x26
#define ETH_TXCR    0x70
#define ETH_RXCR1   0x74
#define ETH_RXCR2   0x76
#define ETH_TXMIR   0x78
#define ETH_RXFHSR  0x7C
#define ETH_RXFHBCR 0x7E
#define ETH_TXQCR   0x80
#define ETH_RXQCR   0x82
#define ETH_TXFDPR  0x84
#define ETH_RXFDPR  0x86
#define ETH_IER     0x90
#define ETH_ISR     0x92
#define ETH_RXFCTR  0x9C
#define ETH_TXNTFSR 0x9E
#define ETH_FCLWR   0xB0
#define ETH_FCHWR   0xB2
#define ETH_CIDER   0xC0
#define ETH_P1MBCR  0xE4
#define ETH_P1MBSR  0xE6
#define ETH_P1SCLMD 0xF4
#define ETH_P1CR    0xF6
#define ETH_P1SR    0xF8

#define RXQCR_RRXEF           (1 << 0)
#define RXQCR_START_DMA       (1 << 3)
#define RXQCR_AUTO_DEQUEUE    (1 << 4)
#define RXQCR_RXFCTE          (1 << 5)
#define RXQCR_RXDBCTE         (1 << 6)
#define RXQCR_RXDTTE          (1 << 7)

#define RX_VALID     (1 << 15)
#define RX_CRC_ERR   (1 << 0)
#define RX_RUNT      (1 << 1)
#define RX_TOO_LONG  (1 << 2)
#define RX_MII_ERR   (1 << 4)

#define TXQCR_ENQUEUE (1 << 0)

// ---- TFTP boot config ----

#define IP(a,b,c,d) (((a)<<24)|((b)<<16)|((c)<<8)|(d))

#define TFTP_SERVER_IP   IP(192,168,1,106)
#define TFTP_FILENAME    "kernel.bin"
#define TFTP_LOAD_ADDR   0x40000000
#define TFTP_CLIENT_PORT 49152
#define TFTP_BLOCK_SIZE  1024

// ---- Network config ----

static const uint8_t MY_MAC[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };

static uint32_t my_ip         = 0;
static uint32_t dhcp_server_ip = 0;
static uint32_t subnet_mask   = 0;
static uint32_t gateway_ip    = 0;

#define DHCP_STATE_INIT       0
#define DHCP_STATE_SELECTING  1
#define DHCP_STATE_REQUESTING 2
#define DHCP_STATE_BOUND      3

static uint8_t  dhcp_state = DHCP_STATE_INIT;
static uint32_t dhcp_xid   = 0xDEADBEEF;
static uint32_t dhcp_delay = 0;

// ---- ARP cache for TFTP server ----

static uint8_t server_mac[6];
static bool server_mac_valid = false;

// ---- TFTP state ----

#define TFTP_IDLE       0
#define TFTP_ARP_WAIT   1
#define TFTP_ACTIVE     2
#define TFTP_DONE       3

static uint8_t  tftp_state = TFTP_IDLE;
static uint16_t tftp_server_port = 0;
static uint16_t tftp_expected_block = 1;
static uint32_t tftp_offset = 0;
static uint32_t tftp_retry_timer = 0;
static uint32_t tftp_retries = 0;

// Cached RXQCR base value
static uint16_t rc_rxqcr = RXQCR_RXFCTE;

// ---- DHCP option codes ----

#define DHCP_OPT_SUBNET_MASK   1
#define DHCP_OPT_ROUTER        3
#define DHCP_OPT_DNS           6
#define DHCP_OPT_REQUESTED_IP 50
#define DHCP_OPT_MSG_TYPE     53
#define DHCP_OPT_SERVER_ID    54
#define DHCP_OPT_PARAM_LIST   55
#define DHCP_OPT_END         255

// ---- Timer (busy-wait, 50 MHz system clock) ----

static void delay(volatile uint32_t n) { while (n--) __asm__ volatile(""); }
static void delay_ms(uint32_t ms) { delay(ms * 12500); }

// ---- SPI controller access (spi_wb buffer-based) ----

static void spi_wait_idle(void) {
  int timeout = 100000;
  while (!(SPI_REG(SPI_STATUS) & SPI_STATUS_IDLE)) {
    if (--timeout <= 0) { printf("SPI: idle timeout!\n"); return; }
  }
}

static void tx_bram_write(const uint8_t *data, uint32_t len) {
  uint32_t i;
  for (i = 0; i + 3 < len; i += 4)
    SPI_TX_BUF[i/4] = data[i] | ((uint32_t)data[i+1]<<8) |
      ((uint32_t)data[i+2]<<16) | ((uint32_t)data[i+3]<<24);
  if (i < len) {
    uint32_t val = 0;
    for (uint32_t j = 0; j < len-i; j++)
      val |= (uint32_t)data[i+j] << (j*8);
    SPI_TX_BUF[i/4] = val;
  }
}

static void tx_bram_write_at(const uint8_t *data, uint32_t offset, uint32_t len) {
  uint32_t i = 0, pos = offset;
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
  while (i + 3 < len) {
    SPI_TX_BUF[pos/4] = data[i] | ((uint32_t)data[i+1]<<8) |
      ((uint32_t)data[i+2]<<16) | ((uint32_t)data[i+3]<<24);
    i += 4; pos += 4;
  }
  if (i < len) {
    uint32_t val = 0;
    for (uint32_t j = 0; i+j < len; j++)
      val |= (uint32_t)data[i+j] << (j*8);
    SPI_TX_BUF[pos/4] = val;
  }
}

static void rx_bram_read(uint8_t *data, uint32_t offset, uint32_t len) {
  uint32_t i = 0, pos = offset;
  while (i < len && (pos & 3)) {
    uint32_t word = SPI_RX_BUF[pos/4];
    data[i++] = (word >> ((pos%4)*8)) & 0xFF;
    pos++;
  }
  while (i + 3 < len) {
    uint32_t word = SPI_RX_BUF[pos/4];
    data[i]   = word & 0xFF;
    data[i+1] = (word>>8) & 0xFF;
    data[i+2] = (word>>16) & 0xFF;
    data[i+3] = (word>>24) & 0xFF;
    i += 4; pos += 4;
  }
  while (i < len) {
    uint32_t word = SPI_RX_BUF[pos/4];
    data[i++] = (word >> ((pos%4)*8)) & 0xFF;
    pos++;
  }
}

static void spi_xfer(uint32_t len) {
  SPI_REG(SPI_CS) = 0;
  SPI_REG(SPI_START) = len;
  spi_wait_idle();
  SPI_REG(SPI_CS) = 1;
}

// ---- KSZ8851 register access (via spi_wb) ----

static uint16_t ksz_read(uint8_t reg) {
  uint8_t be = (reg & 0x2) == 0 ? 0x03 : 0x0C;
  uint8_t cmd[4];
  cmd[0] = (0b00 << 6) | (be << 2) | (reg >> 6);
  cmd[1] = (reg << 2) & 0xF0;
  cmd[2] = 0; cmd[3] = 0;
  tx_bram_write(cmd, 4);
  spi_xfer(4);
  return (SPI_RX_BUF[0] >> 16) & 0xFFFF;
}

static void ksz_write(uint8_t reg, uint16_t val) {
  uint8_t be = (reg & 0x2) == 0 ? 0x03 : 0x0C;
  uint8_t cmd[4];
  cmd[0] = (0b01 << 6) | (be << 2) | (reg >> 6);
  cmd[1] = (reg << 2) & 0xF0;
  cmd[2] = val & 0xFF;
  cmd[3] = val >> 8;
  tx_bram_write(cmd, 4);
  spi_xfer(4);
}

static void ksz_set(uint8_t reg, uint16_t mask) {
  ksz_write(reg, ksz_read(reg) | mask);
}

static __attribute__((unused)) void ksz_clear(uint8_t reg, uint16_t mask) {
  ksz_write(reg, ksz_read(reg) & ~mask);
}

// ---- TX: send raw frame via KSZ8851 FIFO ----

static void ksz_tx(const uint8_t *buf, uint16_t len) {
  int timeout = 100000;
  while (timeout-- > 0) {
    if ((ksz_read(ETH_TXMIR) & 0x0FFF) >= len + 4) break;
  }
  if (timeout <= 0) { printf("TX FIFO full!\n"); return; }

  ksz_write(ETH_TXFDPR, 0x4000);
  ksz_write(ETH_RXQCR, rc_rxqcr | RXQCR_START_DMA);

  uint32_t pad = (-len) & 0x3;
  uint32_t total = 1 + 4 + len + pad;

  uint8_t hdr[5];
  hdr[0] = 0xC0;
  hdr[1] = 0x00;
  hdr[2] = 0x80;
  hdr[3] = len & 0xFF;
  hdr[4] = len >> 8;
  tx_bram_write(hdr, 5);
  tx_bram_write_at(buf, 5, len);
  if (pad > 0) {
    uint8_t zeros[4] = {0, 0, 0, 0};
    tx_bram_write_at(zeros, 5 + len, pad);
  }

  SPI_REG(SPI_CS) = 0;
  SPI_REG(SPI_START) = total;
  spi_wait_idle();
  SPI_REG(SPI_CS) = 1;

  ksz_write(ETH_RXQCR, rc_rxqcr);
  ksz_set(ETH_TXQCR, TXQCR_ENQUEUE);
}

// ---- IP checksum ----

static uint16_t ip_checksum(const uint8_t *p, uint16_t len) {
  uint32_t s = 0;
  while (len > 1) {
    s += (p[0] << 8) | p[1];
    p += 2;
    len -= 2;
  }
  if (len) s += p[0] << 8;
  while (s >> 16) s = (s & 0xFFFF) + (s >> 16);
  return ~s;
}

// ---- Print helpers ----

static void print_ip(uint32_t ip) {
  printf("%ld.%ld.%ld.%ld",
    (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
    (ip >> 8) & 0xFF, ip & 0xFF);
}

// ---- Build and send DHCP packet (Discover or Request) ----

static void send_dhcp_packet(uint8_t msg_type, uint32_t requested_ip, uint32_t server_id) {
  uint8_t pkt[400];
  memset(pkt, 0, sizeof(pkt));
  uint8_t *p = pkt;

  // Ethernet header
  memset(p, 0xFF, 6); p += 6;
  memcpy(p, MY_MAC, 6); p += 6;
  *p++ = 0x08; *p++ = 0x00;

  // IP header (20 bytes)
  uint8_t *ip_hdr = p;
  *p++ = 0x45;
  *p++ = 0x00;
  p += 2;                // Total length (fill later)
  *p++ = 0x00; *p++ = 0x00;
  *p++ = 0x00; *p++ = 0x00;
  *p++ = 64;
  *p++ = 17;
  *p++ = 0x00; *p++ = 0x00;
  *p++ = 0; *p++ = 0; *p++ = 0; *p++ = 0;
  *p++ = 0xFF; *p++ = 0xFF; *p++ = 0xFF; *p++ = 0xFF;

  // UDP header (8 bytes)
  uint8_t *udp_hdr = p;
  *p++ = 0x00; *p++ = 68;
  *p++ = 0x00; *p++ = 67;
  p += 2;
  *p++ = 0x00; *p++ = 0x00;

  // DHCP message (240 byte header)
  uint8_t *dhcp_start = p;
  *p++ = 1;
  *p++ = 1;
  *p++ = 6;
  *p++ = 0;
  *p++ = (dhcp_xid >> 24) & 0xFF;
  *p++ = (dhcp_xid >> 16) & 0xFF;
  *p++ = (dhcp_xid >> 8) & 0xFF;
  *p++ = dhcp_xid & 0xFF;
  *p++ = 0; *p++ = 0;
  *p++ = 0x80; *p++ = 0x00;
  memset(p, 0, 16); p += 16;
  memcpy(p, MY_MAC, 6);
  p += 16;
  memset(p, 0, 64); p += 64;
  memset(p, 0, 128); p += 128;

  // Magic cookie
  *p++ = 99; *p++ = 130; *p++ = 83; *p++ = 99;

  // Option 53: DHCP Message Type
  *p++ = DHCP_OPT_MSG_TYPE; *p++ = 1; *p++ = msg_type;

  if (msg_type == 3 && requested_ip != 0) {
    *p++ = DHCP_OPT_REQUESTED_IP; *p++ = 4;
    *p++ = (requested_ip >> 24) & 0xFF;
    *p++ = (requested_ip >> 16) & 0xFF;
    *p++ = (requested_ip >> 8) & 0xFF;
    *p++ = requested_ip & 0xFF;

    if (server_id != 0) {
      *p++ = DHCP_OPT_SERVER_ID; *p++ = 4;
      *p++ = (server_id >> 24) & 0xFF;
      *p++ = (server_id >> 16) & 0xFF;
      *p++ = (server_id >> 8) & 0xFF;
      *p++ = server_id & 0xFF;
    }
  }

  // Option 55: Parameter Request List
  *p++ = DHCP_OPT_PARAM_LIST; *p++ = 3;
  *p++ = DHCP_OPT_SUBNET_MASK;
  *p++ = DHCP_OPT_ROUTER;
  *p++ = DHCP_OPT_DNS;

  *p++ = DHCP_OPT_END;

  // Pad DHCP to at least 300 bytes
  uint16_t dhcp_len = p - dhcp_start;
  if (dhcp_len < 300) {
    memset(p, 0, 300 - dhcp_len);
    p += 300 - dhcp_len;
    dhcp_len = 300;
  }

  // Fill in lengths
  uint16_t udp_len = 8 + dhcp_len;
  uint16_t ip_len  = 20 + udp_len;
  ip_hdr[2] = ip_len >> 8;
  ip_hdr[3] = ip_len & 0xFF;
  udp_hdr[4] = udp_len >> 8;
  udp_hdr[5] = udp_len & 0xFF;

  // IP checksum
  uint16_t csum = ip_checksum(ip_hdr, 20);
  ip_hdr[10] = csum >> 8;
  ip_hdr[11] = csum & 0xFF;

  uint16_t total = 14 + ip_len;
  ksz_tx(pkt, total);

  printf("TX DHCP %s\n", msg_type == 1 ? "DISCOVER" : "REQUEST");
}

// ---- Parse DHCP reply (Offer or ACK) ----

static void handle_dhcp(const uint8_t *dhcp_data, uint16_t dhcp_len) {
  if (dhcp_len < 240) return;

  uint32_t rxid = ((uint32_t)dhcp_data[4] << 24) | ((uint32_t)dhcp_data[5] << 16) |
                  ((uint32_t)dhcp_data[6] << 8) | dhcp_data[7];
  if (rxid != dhcp_xid) {
    printf("  DHCP xid mismatch\n");
    return;
  }

  uint32_t offered_ip = ((uint32_t)dhcp_data[16] << 24) | ((uint32_t)dhcp_data[17] << 16) |
                        ((uint32_t)dhcp_data[18] << 8) | dhcp_data[19];

  const uint8_t *opt = dhcp_data + 240;
  const uint8_t *end = dhcp_data + dhcp_len;
  uint8_t msg_type = 0;
  uint32_t server_id = 0;
  uint32_t subnet = 0;
  uint32_t router = 0;

  while (opt < end) {
    uint8_t otype = *opt++;
    if (otype == DHCP_OPT_END) break;
    if (opt >= end) break;
    uint8_t olen = *opt++;
    if (opt + olen > end) break;

    switch (otype) {
      case DHCP_OPT_MSG_TYPE:
        if (olen == 1) msg_type = opt[0];
        break;
      case DHCP_OPT_SERVER_ID:
        if (olen == 4) server_id = ((uint32_t)opt[0] << 24) | ((uint32_t)opt[1] << 16) |
                                   ((uint32_t)opt[2] << 8) | opt[3];
        break;
      case DHCP_OPT_SUBNET_MASK:
        if (olen == 4) subnet = ((uint32_t)opt[0] << 24) | ((uint32_t)opt[1] << 16) |
                                ((uint32_t)opt[2] << 8) | opt[3];
        break;
      case DHCP_OPT_ROUTER:
        if (olen >= 4) router = ((uint32_t)opt[0] << 24) | ((uint32_t)opt[1] << 16) |
                                ((uint32_t)opt[2] << 8) | opt[3];
        break;
    }
    opt += olen;
  }

  printf("  DHCP msg_type=%d state=%d\n", msg_type, dhcp_state);

  if (msg_type == 2 && dhcp_state == DHCP_STATE_SELECTING) {
    printf("  DHCP OFFER: ");
    print_ip(offered_ip);
    printf("\n");
    dhcp_state = DHCP_STATE_REQUESTING;
    dhcp_delay = 1000000;
    send_dhcp_packet(3, offered_ip, server_id);
  }
  else if (msg_type == 5 && dhcp_state == DHCP_STATE_REQUESTING) {
    my_ip = offered_ip;
    dhcp_server_ip = server_id;
    subnet_mask = subnet;
    gateway_ip = router;
    dhcp_state = DHCP_STATE_BOUND;

    printf("  DHCP ACK: IP=");
    print_ip(my_ip);
    printf(" mask=");
    print_ip(subnet_mask);
    printf(" gw=");
    print_ip(gateway_ip);
    printf("\n");
  }
}

// ---- Send ARP reply ----

static void send_arp_reply(const uint8_t *target_mac, const uint8_t *target_ip_bytes) {
  uint8_t pkt[42];
  uint8_t *p = pkt;

  memcpy(p, target_mac, 6); p += 6;
  memcpy(p, MY_MAC, 6); p += 6;
  *p++ = 0x08; *p++ = 0x06;

  *p++ = 0x00; *p++ = 0x01;
  *p++ = 0x08; *p++ = 0x00;
  *p++ = 6;
  *p++ = 4;
  *p++ = 0x00; *p++ = 0x02;

  memcpy(p, MY_MAC, 6); p += 6;
  *p++ = (my_ip >> 24) & 0xFF;
  *p++ = (my_ip >> 16) & 0xFF;
  *p++ = (my_ip >> 8) & 0xFF;
  *p++ = my_ip & 0xFF;

  memcpy(p, target_mac, 6); p += 6;
  memcpy(p, target_ip_bytes, 4); p += 4;

  ksz_tx(pkt, 42);
  printf("  TX ARP reply\n");
}

// ---- Send ARP request ----

static void send_arp_request(uint32_t target_ip) {
  uint8_t pkt[42];
  uint8_t *p = pkt;

  memset(p, 0xFF, 6); p += 6;
  memcpy(p, MY_MAC, 6); p += 6;
  *p++ = 0x08; *p++ = 0x06;

  *p++ = 0x00; *p++ = 0x01;
  *p++ = 0x08; *p++ = 0x00;
  *p++ = 6;
  *p++ = 4;
  *p++ = 0x00; *p++ = 0x01;

  memcpy(p, MY_MAC, 6); p += 6;
  *p++ = (my_ip >> 24) & 0xFF;
  *p++ = (my_ip >> 16) & 0xFF;
  *p++ = (my_ip >> 8) & 0xFF;
  *p++ = my_ip & 0xFF;

  memset(p, 0x00, 6); p += 6;
  *p++ = (target_ip >> 24) & 0xFF;
  *p++ = (target_ip >> 16) & 0xFF;
  *p++ = (target_ip >> 8) & 0xFF;
  *p++ = target_ip & 0xFF;

  ksz_tx(pkt, 42);
  printf("TX ARP request for ");
  print_ip(target_ip);
  printf("\n");
}

// ---- Generic UDP TX ----

static void send_udp(const uint8_t *dst_mac, uint32_t dst_ip,
                     uint16_t src_port, uint16_t dst_port,
                     const uint8_t *data, uint16_t data_len) {
  uint16_t udp_len = 8 + data_len;
  uint16_t ip_len  = 20 + udp_len;
  uint16_t eth_total = 14 + ip_len;

  uint8_t pkt[1514];
  if (eth_total > sizeof(pkt)) return;
  uint8_t *p = pkt;

  memcpy(p, dst_mac, 6); p += 6;
  memcpy(p, MY_MAC, 6); p += 6;
  *p++ = 0x08; *p++ = 0x00;

  uint8_t *ip_hdr = p;
  *p++ = 0x45; *p++ = 0x00;
  *p++ = ip_len >> 8; *p++ = ip_len & 0xFF;
  *p++ = 0; *p++ = 0;
  *p++ = 0; *p++ = 0;
  *p++ = 64;
  *p++ = 17;
  *p++ = 0; *p++ = 0;
  *p++ = (my_ip >> 24) & 0xFF;
  *p++ = (my_ip >> 16) & 0xFF;
  *p++ = (my_ip >> 8) & 0xFF;
  *p++ = my_ip & 0xFF;
  *p++ = (dst_ip >> 24) & 0xFF;
  *p++ = (dst_ip >> 16) & 0xFF;
  *p++ = (dst_ip >> 8) & 0xFF;
  *p++ = dst_ip & 0xFF;

  uint16_t csum = ip_checksum(ip_hdr, 20);
  ip_hdr[10] = csum >> 8;
  ip_hdr[11] = csum & 0xFF;

  *p++ = src_port >> 8; *p++ = src_port & 0xFF;
  *p++ = dst_port >> 8; *p++ = dst_port & 0xFF;
  *p++ = udp_len >> 8;  *p++ = udp_len & 0xFF;
  *p++ = 0; *p++ = 0;

  memcpy(p, data, data_len);

  ksz_tx(pkt, eth_total);
}

// ---- TFTP RRQ ----

static void send_tftp_rrq(void) {
  uint8_t rrq[80];
  uint8_t *p = rrq;
  *p++ = 0; *p++ = 1;
  const char *fn = TFTP_FILENAME;
  while (*fn) *p++ = *fn++;
  *p++ = 0;
  const char *mode = "octet";
  while (*mode) *p++ = *mode++;
  *p++ = 0;
  const char *opt = "blksize";
  while (*opt) *p++ = *opt++;
  *p++ = 0;
  const char *val = "1024";
  while (*val) *p++ = *val++;
  *p++ = 0;

  send_udp(server_mac, TFTP_SERVER_IP, TFTP_CLIENT_PORT, 69, rrq, p - rrq);
  printf("TX TFTP RRQ \"%s\" blksize=1024\n", TFTP_FILENAME);
}

// ---- TFTP ACK ----

static void send_tftp_ack(uint16_t block) {
  uint8_t ack[4];
  ack[0] = 0; ack[1] = 4;
  ack[2] = block >> 8;
  ack[3] = block & 0xFF;
  send_udp(server_mac, TFTP_SERVER_IP, TFTP_CLIENT_PORT, tftp_server_port, ack, 4);
}

// ---- Send ICMP echo reply ----

static void send_icmp_reply(const uint8_t *src_mac, const uint8_t *src_ip_bytes,
                            const uint8_t *icmp_data, uint16_t icmp_len) {
  uint16_t ip_total = 20 + icmp_len;
  uint16_t eth_total = 14 + ip_total;

  uint8_t pkt[1514];
  if (eth_total > sizeof(pkt)) return;
  uint8_t *p = pkt;

  memcpy(p, src_mac, 6); p += 6;
  memcpy(p, MY_MAC, 6); p += 6;
  *p++ = 0x08; *p++ = 0x00;

  uint8_t *ip_hdr = p;
  *p++ = 0x45; *p++ = 0x00;
  *p++ = ip_total >> 8; *p++ = ip_total & 0xFF;
  *p++ = 0; *p++ = 0;
  *p++ = 0; *p++ = 0;
  *p++ = 64;
  *p++ = 1;
  *p++ = 0; *p++ = 0;
  *p++ = (my_ip >> 24) & 0xFF;
  *p++ = (my_ip >> 16) & 0xFF;
  *p++ = (my_ip >> 8) & 0xFF;
  *p++ = my_ip & 0xFF;
  memcpy(p, src_ip_bytes, 4); p += 4;

  uint16_t csum = ip_checksum(ip_hdr, 20);
  ip_hdr[10] = csum >> 8;
  ip_hdr[11] = csum & 0xFF;

  uint8_t *icmp_hdr = p;
  *p++ = 0;
  *p++ = 0;
  *p++ = 0; *p++ = 0;
  memcpy(p, icmp_data + 4, icmp_len - 4);
  p += icmp_len - 4;

  uint16_t icmp_csum = ip_checksum(icmp_hdr, icmp_len);
  icmp_hdr[2] = icmp_csum >> 8;
  icmp_hdr[3] = icmp_csum & 0xFF;

  ksz_tx(pkt, eth_total);
  printf("  TX ICMP reply\n");
}

// ---- RX: read and process one poll cycle ----

// rxbuf in HyperRAM to save SRAM BSS (only 6KB SRAM total)
#define RXBUF_ADDR 0x407FE000
static uint8_t * const rxbuf = (uint8_t *)RXBUF_ADDR;

static void rx_poll(void) {
  uint16_t isr = ksz_read(ETH_ISR);
  if (isr)
    ksz_write(ETH_ISR, isr);

  // Handle RX process stopped
  if (isr & (1 << 8)) {
    uint16_t rxcr1 = ksz_read(ETH_RXCR1);
    if (!(rxcr1 & (1 << 0)))
      ksz_write(ETH_RXCR1, rxcr1 | (1 << 0));
  }

  if (!(isr & (1 << 13)))
    return;

  uint16_t frames = ksz_read(ETH_RXFCTR) >> 8;
  if (!frames) return;

  for (uint16_t f = 0; f < frames; f++) {
    uint16_t status = ksz_read(ETH_RXFHSR);
    uint16_t len    = ksz_read(ETH_RXFHBCR) & 0xFFF;

    bool valid = (status & RX_VALID) &&
                 !(status & (RX_CRC_ERR | RX_RUNT | RX_TOO_LONG | RX_MII_ERR));

    if (!valid || len <= 4 || len > 1536) {
      ksz_write(ETH_RXQCR, rc_rxqcr | RXQCR_RRXEF);
      continue;
    }

    uint16_t frame_len = len - 4;
    uint16_t rxalign = (len + 3) & ~3;

    ksz_write(ETH_RXFDPR, 0x4000);
    ksz_write(ETH_RXQCR, rc_rxqcr | RXQCR_START_DMA);

    uint32_t fifo_len = rxalign + 8;
    uint32_t total = 1 + fifo_len;

    SPI_TX_BUF[0] = 0x80;
    for (uint32_t w = 1; w < (total+3)/4; w++)
      SPI_TX_BUF[w] = 0;

    SPI_REG(SPI_CS) = 0;
    SPI_REG(SPI_START) = total;
    spi_wait_idle();

    rx_bram_read(rxbuf, 9, frame_len);

    SPI_REG(SPI_CS) = 1;

    ksz_write(ETH_RXQCR, rc_rxqcr | RXQCR_RRXEF);

    // Skip our own TX (loopback)
    if (memcmp(rxbuf + 6, MY_MAC, 6) == 0)
      continue;

    // ---- Parse frame ----
    uint8_t *p = rxbuf;
    p += 6;  // skip dst_mac
    uint8_t *src_mac = p; p += 6;
    uint16_t ethertype = (p[0] << 8) | p[1]; p += 2;

    if (ethertype == 0x0806) {
      // ---- ARP ----
      if (frame_len < 42) continue;
      uint16_t operation = (p[6] << 8) | p[7];
      uint8_t *sender_mac = p + 8;
      uint8_t *sender_ip  = p + 14;
      uint8_t *target_ip  = p + 24;

      uint32_t tip = ((uint32_t)target_ip[0] << 24) | ((uint32_t)target_ip[1] << 16) |
                     ((uint32_t)target_ip[2] << 8) | target_ip[3];

      if (operation == 1 && my_ip != 0 && tip == my_ip) {
        printf("RX ARP who-has ");
        print_ip(my_ip);
        printf("\n");
        send_arp_reply(sender_mac, sender_ip);
      }
      else if (operation == 2) {
        uint32_t sip = ((uint32_t)sender_ip[0] << 24) | ((uint32_t)sender_ip[1] << 16) |
                       ((uint32_t)sender_ip[2] << 8) | sender_ip[3];
        if (sip == TFTP_SERVER_IP && !server_mac_valid) {
          memcpy(server_mac, sender_mac, 6);
          server_mac_valid = true;
          printf("  ARP: resolved server MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
            server_mac[0], server_mac[1], server_mac[2],
            server_mac[3], server_mac[4], server_mac[5]);
        }
      }
    }
    else if (ethertype == 0x0800) {
      // ---- IPv4 ----
      if (frame_len < 34) continue;
      uint8_t ihl = (p[0] & 0x0F) * 4;
      uint16_t ip_len = (p[2] << 8) | p[3];
      uint8_t protocol = p[9];
      uint8_t *src_ip = p + 12;
      uint8_t *dst_ip = p + 16;

      uint32_t dip = ((uint32_t)dst_ip[0] << 24) | ((uint32_t)dst_ip[1] << 16) |
                     ((uint32_t)dst_ip[2] << 8) | dst_ip[3];

      bool for_us = (my_ip != 0 && dip == my_ip);
      bool is_broadcast = (dip == 0xFFFFFFFF);

      uint8_t *payload = p + ihl;

      if (for_us && protocol == 1) {
        // ---- ICMP ----
        uint8_t icmp_type = payload[0];
        if (icmp_type == 8 && ip_len > (uint16_t)(ihl + 8)) {
          uint16_t icmp_len = ip_len - ihl;
          uint16_t ping_id  = (payload[4] << 8) | payload[5];
          uint16_t ping_seq = (payload[6] << 8) | payload[7];

          printf("RX PING from %d.%d.%d.%d id=%04x seq=%04x\n",
            src_ip[0], src_ip[1], src_ip[2], src_ip[3], ping_id, ping_seq);
          send_icmp_reply(src_mac, src_ip, payload, icmp_len);
        }
      }
      else if ((for_us || is_broadcast || my_ip == 0) && protocol == 17) {
        // ---- UDP ----
        uint16_t src_port = (payload[0] << 8) | payload[1];
        uint16_t dst_port = (payload[2] << 8) | payload[3];
        uint16_t udp_len  = (payload[4] << 8) | payload[5];
        uint8_t *udp_data = payload + 8;
        uint16_t udp_payload_len = udp_len - 8;

        if (dst_port == 68 && src_port == 67 && dhcp_state != DHCP_STATE_BOUND) {
          printf("RX DHCP from %d.%d.%d.%d\n",
            src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
          handle_dhcp(udp_data, udp_payload_len);
        }
        else if (dst_port == TFTP_CLIENT_PORT && tftp_state == TFTP_ACTIVE) {
          if (udp_payload_len < 4) continue;
          uint16_t opcode = (udp_data[0] << 8) | udp_data[1];

          if (opcode == 5) {
            uint16_t errcode = (udp_data[2] << 8) | udp_data[3];
            printf("TFTP ERROR %d: ", errcode);
            if (udp_payload_len > 4) {
              for (uint16_t i = 4; i < udp_payload_len && udp_data[i]; i++)
                printf("%c", udp_data[i]);
            }
            printf("\nRetrying in 30s...\n");
            delay_ms(30000);
            tftp_state = TFTP_IDLE;
          }
          else if (opcode == 6) {
            tftp_server_port = src_port;
            send_tftp_ack(0);
            tftp_retry_timer = 0;
            printf("TFTP: OACK received, blksize negotiated\n");
          }
          else if (opcode == 3) {
            uint16_t block = (udp_data[2] << 8) | udp_data[3];
            uint16_t payload_len = udp_payload_len - 4;

            if (tftp_expected_block == 1 && block == 1)
              tftp_server_port = src_port;

            if (block == tftp_expected_block) {
              uint8_t *dst = (uint8_t *)(TFTP_LOAD_ADDR + tftp_offset);
              memcpy(dst, udp_data + 4, payload_len);
              tftp_offset += payload_len;

              send_tftp_ack(block);
              tftp_expected_block++;
              tftp_retry_timer = 0;

              if ((block & 63) == 0) {
                printf("TFTP: %ld bytes\n", tftp_offset);
              }

              if (payload_len < TFTP_BLOCK_SIZE) {
                tftp_state = TFTP_DONE;
                printf("TFTP: transfer complete, %ld bytes at 0x%08lx\n",
                  tftp_offset, (unsigned long)TFTP_LOAD_ADDR);
              }
            }
            else if (block == tftp_expected_block - 1) {
              send_tftp_ack(block);
            }
          }
        }
      }
    }
  }
}

// ---- Diagnostic ----

static void pr_reg(const char *name, uint8_t reg) {
  printf("%s=0x%04x ", name, ksz_read(reg));
}

// ---- Entry point (called from BIOS boot_sequence) ----

void spieth_tftp_boot(void) {
  printf("\n=== KSZ8851 DHCP + TFTP Boot (LiteX spi_wb) ===\n");

  uint16_t cider = 0;

  for (int attempt = 0; attempt < 3; attempt++) {
    // Software reset of SPI controller
    SPI_REG(SPI_CONTROL) = (1u << 31);
    delay_ms(1);
    // Configure: MSB-first, CPOL=0, CPHA=0, half_clk_period=1 (25MHz)
    SPI_REG(SPI_CFG) = (1 << 29) | 1;
    SPI_REG(SPI_CS) = 1;
    spi_wait_idle();
    delay_ms(1);

    printf("SPI: CFG=0x%08lx STATUS=0x%08lx\n",
      (unsigned long)SPI_REG(SPI_CFG),
      (unsigned long)SPI_REG(SPI_STATUS));

    // Pulse CS to reset KSZ8851 SPI state machine
    for (int i = 0; i < 10; i++) {
      SPI_REG(SPI_CS) = 0;
      delay(500);
      SPI_REG(SPI_CS) = 1;
      delay(500);
    }
    delay_ms(10);

    // Software reset of KSZ8851
    printf("Resetting KSZ8851 (attempt %d)...\n", attempt);
    ksz_write(ETH_GRR, 0x0001);
    delay_ms(100);
    ksz_write(ETH_GRR, 0x0000);
    delay_ms(100);

    cider = ksz_read(ETH_CIDER);
    printf("Chip ID: 0x%04x\n", cider);
    if ((cider & 0xFFF0) == 0x8870)
      break;

    printf("Bad chip ID, retrying...\n");
    delay_ms(100);
  }

  if ((cider & 0xFFF0) != 0x8870) {
    printf("FATAL: Bad chip ID after 3 attempts\n");
    return;
  }

  // MAC address
  ksz_write(ETH_MARH, 0x0200);
  ksz_write(ETH_MARM, 0x0000);
  ksz_write(ETH_MARL, 0x0001);
  printf("MAC: 02:00:00:00:00:01\n");

  // TX init
  ksz_write(ETH_TXFDPR, 0x4000);
  ksz_write(ETH_TXCR, 0x00EE);

  // RX init
  ksz_write(ETH_RXFDPR, 0x4000);
  ksz_write(ETH_RXFCTR, 0x0001);
  ksz_write(ETH_RXCR1, 0x7CE0);
  ksz_write(ETH_RXCR2, 0x009C);
  ksz_write(ETH_RXQCR, rc_rxqcr);

  // PHY
  ksz_set(ETH_P1CR, 1 << 13);

  // Flow control
  ksz_write(ETH_FCLWR, 0x0600);
  ksz_write(ETH_FCHWR, 0x0400);

  // Interrupts (polling only)
  ksz_write(ETH_ISR, 0xFFFF);
  ksz_write(ETH_IER, 0xE000);

  // Enable TX and RX
  ksz_set(ETH_TXCR, 1 << 0);
  ksz_set(ETH_RXCR1, 1 << 0);

  printf("Init done. ");
  pr_reg("RXCR1", ETH_RXCR1);
  pr_reg("RXCR2", ETH_RXCR2);
  pr_reg("RXQCR", ETH_RXQCR);
  pr_reg("TXCR", ETH_TXCR);
  printf("\n");

  // Wait for link
  printf("Waiting for link...\n");
  for (int i = 0; i < 50; i++) {
    if (ksz_read(ETH_P1SR) & (1 << 5)) {
      printf("Link UP!\n");
      break;
    }
    printf(".");
    delay_ms(100);
  }
  printf("\n");

  // ---- Main loop: DHCP then TFTP then boot ----

  uint32_t loop_count = 0;

  while (1) {
    rx_poll();

    // DHCP state machine
    if (dhcp_state == DHCP_STATE_INIT) {
      if (dhcp_delay == 0) {
        dhcp_delay = 500000;
        dhcp_state = DHCP_STATE_SELECTING;
        send_dhcp_packet(1, 0, 0);
      } else {
        dhcp_delay--;
      }
    }
    else if (dhcp_state == DHCP_STATE_SELECTING || dhcp_state == DHCP_STATE_REQUESTING) {
      if (dhcp_delay == 0) {
        dhcp_delay = 1000000;
        dhcp_state = DHCP_STATE_SELECTING;
        send_dhcp_packet(1, 0, 0);
      } else {
        dhcp_delay--;
      }
    }

    // ---- TFTP boot state machine ----
    if (dhcp_state == DHCP_STATE_BOUND) {
      if (tftp_state == TFTP_IDLE) {
        printf("\n--- TFTP boot: resolving server ");
        print_ip(TFTP_SERVER_IP);
        printf(" ---\n");
        tftp_state = TFTP_ARP_WAIT;
        tftp_retry_timer = 0;
        tftp_retries = 0;
        send_arp_request(TFTP_SERVER_IP);
      }
      else if (tftp_state == TFTP_ARP_WAIT) {
        if (server_mac_valid) {
          printf("Sending TFTP RRQ...\n");
          tftp_expected_block = 1;
          tftp_offset = 0;
          tftp_server_port = 0;
          tftp_state = TFTP_ACTIVE;
          send_tftp_rrq();
        } else if (++tftp_retry_timer >= 500000) {
          tftp_retry_timer = 0;
          if (++tftp_retries > 10) {
            printf("TFTP: ARP resolve failed, giving up\n");
            return;
          } else {
            send_arp_request(TFTP_SERVER_IP);
          }
        }
      }
      else if (tftp_state == TFTP_ACTIVE) {
        if (++tftp_retry_timer >= 2000000) {
          tftp_retry_timer = 0;
          if (++tftp_retries > 10) {
            printf("TFTP: timeout, giving up\n");
            tftp_state = TFTP_IDLE;
          } else if (tftp_expected_block == 1) {
            printf("TFTP: retransmit RRQ\n");
            send_tftp_rrq();
          } else {
            printf("TFTP: retransmit ACK\n");
            send_tftp_ack(tftp_expected_block - 1);
          }
        }
      }
      else if (tftp_state == TFTP_DONE && tftp_offset > 0) {
        printf("\n=== Booting from 0x%08lx ===\n\n", (unsigned long)TFTP_LOAD_ADDR);

        delay_ms(10);  // Flush UART
        boot(0, 0, 0, TFTP_LOAD_ADDR);
        // boot() does not return
      }
    }

    // Periodic status
    if (++loop_count >= 200000) {
      loop_count = 0;
      printf("state=%d", dhcp_state);
      if (my_ip) {
        printf(" ip=");
        print_ip(my_ip);
      }
      printf(" ISR=0x%04x RXFCTR=0x%04x\n",
        ksz_read(ETH_ISR), ksz_read(ETH_RXFCTR));
    }
  }
}

#endif /* SPIETH_BASE */
