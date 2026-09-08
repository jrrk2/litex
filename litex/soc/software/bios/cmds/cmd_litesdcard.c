// SPDX-License-Identifier: BSD-Source-Code

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <generated/csr.h>
#include <system.h>

#include <liblitesdcard/sdcard.h>

#include "../command.h"
#include "../helpers.h"

/**
 * Command "sdcard_detect"
 *
 * Detect SDCard
 *
 */
#ifdef CSR_SDCARD_PHY_CARD_DETECT_ADDR
static void sdcard_detect_handler(int nb_params, char **params)
{
	uint8_t cd = sdcard_phy_card_detect_read();
	printf("SDCard %sinserted.\n", cd ? "not " : "");
}

define_command(sdcard_detect, sdcard_detect_handler, "Detect SDCard", LITESDCARD_CMDS);
#endif

/*
 * Step-by-step SD bring-up commands.
 *
 * SDCARD_DEBUG traces every command including the ACMD41 polling loop, which
 * floods a slow console and leaves no room to type.  These do one step each
 * and answer in one line, so a bring-up can be driven by hand.
 */
#ifdef CSR_SDCARD_CORE_CMD_EVENT_ADDR

/* sdc <op> <arg> [rsp] -- issue one command.  rsp: 0 none, 1 short (default),
 * 2 long, 3 short-busy.  Prints the event byte and all four response words. */
static void sdc_handler(int nb_params, char **params)
{
	char *c;
	uint32_t r[SD_CMD_RESPONSE_SIZE/4];
	unsigned int op, arg, rsp = 1, evt;
	int i;

	if (nb_params < 2) {
		printf("sdc <op> <arg> [rsp 0=none 1=short 2=long 3=busy]\n");
		return;
	}
	op  = strtoul(params[0], &c, 0);
	arg = strtoul(params[1], &c, 0);
	if (nb_params > 2)
		rsp = strtoul(params[2], &c, 0);

	sdcard_core_cmd_argument_write(arg);
	sdcard_core_cmd_command_write((op << 8) | rsp);
	sdcard_core_cmd_send_write(1);

	for (i = 0; i < 100000; i++) {
		evt = sdcard_core_cmd_event_read();
		if (evt & 0x1)
			break;
		busy_wait_us(10);
	}
	csr_rd_buf_uint32(CSR_SDCARD_CORE_CMD_RESPONSE_ADDR, r, SD_CMD_RESPONSE_SIZE/4);
	printf("evt=%02x%s%s %08lx %08lx %08lx %08lx\n", evt & 0xff,
	       (evt & 0x4) ? " TIMEOUT" : "", (evt & 0x8) ? " CRC" : "",
	       (unsigned long)r[0], (unsigned long)r[1],
	       (unsigned long)r[2], (unsigned long)r[3]);
}
define_command(sdc, sdc_handler, "Send one raw SD command", LITESDCARD_CMDS);

/* sdst -- the PHY and core state in one line. */
static void sdst_handler(int nb_params, char **params)
{
	printf("cd=%d div=%ld set=%ld cmdevt=%02lx dataevt=%02lx\n",
	       (int)sdcard_phy_card_detect_read(),
	       (unsigned long)sdcard_phy_clocker_divider_read(),
	       (unsigned long)sdcard_phy_settings_read(),
	       (unsigned long)sdcard_core_cmd_event_read(),
	       (unsigned long)sdcard_core_data_event_read());
}
define_command(sdst, sdst_handler, "SD PHY/core state, one line", LITESDCARD_CMDS);

/* sdblk <block> -- read one 512B block by DMA.  Prints the data event and the
 * first 16 bytes, which is enough to tell a real sector from zeros or noise. */
static void sdblk_handler(int nb_params, char **params)
{
	static uint8_t buf[512] __attribute__((aligned(8)));
	char *c;
	unsigned int blk, evt;
	int i;

	if (nb_params < 1) { printf("sdblk <block>\n"); return; }
	blk = strtoul(params[0], &c, 0);

	memset(buf, 0xa5, sizeof(buf));
	flush_cpu_dcache();

	sdcard_core_block_length_write(512);
	sdcard_core_block_count_write(1);
	sdcard_block2mem_dma_base_write((uint64_t)(uintptr_t)buf);
	sdcard_block2mem_dma_length_write(sizeof(buf));
	sdcard_block2mem_dma_enable_write(1);

	sdcard_core_cmd_argument_write(blk);
	sdcard_core_cmd_command_write((17 << 8) | (1 << 5) | 1);
	sdcard_core_cmd_send_write(1);

	for (i = 0; i < 100000; i++) {
		evt = sdcard_core_data_event_read();
		if (evt & 0x1)
			break;
		busy_wait_us(10);
	}
	flush_cpu_dcache();
	printf("dataevt=%02x%s%s ", evt & 0xff,
	       (evt & 0x4) ? " TIMEOUT" : "", (evt & 0x8) ? " CRC" : "");
	for (i = 0; i < 16; i++)
		printf("%02x", buf[i]);
	printf("\n");
}
define_command(sdblk, sdblk_handler, "Read one 512B block, show 16 bytes", LITESDCARD_CMDS);

#endif

/**
 * Command "sdcard_init"
 *
 * Initialize SDCard
 *
 */
#ifdef CSR_SDCARD_BASE
static void sdcard_init_handler(int nb_params, char **params)
{
	bios_print_status("Initialize SDCard", sdcard_init());
}

define_command(sdcard_init, sdcard_init_handler, "Initialize SDCard", LITESDCARD_CMDS);
#endif

/**
 * Command "sdcard_freq"
 *
 * Set SDCard clock frequency
 *
 */
#ifdef CSR_SDCARD_BASE
static void sdcard_freq_handler(int nb_params, char **params)
{
	unsigned int freq;
	char *c;

	if (nb_params < 1) {
		printf("sdcard_freq <freq>\n");
		return;
	}

	freq = strtoul(params[0], &c, 0);
	if (*c != 0) {
		printf("Error: invalid freq\n");
		return;
	}
	sdcard_set_clk_freq(freq, 1);
}

define_command(sdcard_freq, sdcard_freq_handler, "Set SDCard clock freq", LITESDCARD_CMDS);
#endif

/**
 * Command "sdcard_read"
 *
 * Perform SDCard block read
 *
 */
#ifdef CSR_SDCARD_BLOCK2MEM_DMA_BASE_ADDR
static void sdcard_read_handler(int nb_params, char **params)
{
	unsigned int block;
	unsigned int count = 1;
	unsigned long addr;
	char *c;
	uint8_t buf[512];
	uint8_t *dst = buf;

	if (nb_params < 1) {
		printf("sdcard_read <block> [addr] [count]\n");
		return;
	}

	block = strtoul(params[0], &c, 0);
	if (*c != 0) {
		printf("Error: invalid block number\n");
		return;
	}
	if (nb_params >= 2) {
		addr = strtoul(params[1], &c, 0);
		if (*c != 0) {
			printf("Error: invalid destination address\n");
			return;
		}
		dst = (uint8_t *)(uintptr_t)addr;
	}
	if (nb_params >= 3) {
		count = strtoul(params[2], &c, 0);
		if (*c != 0) {
			printf("Error: invalid count\n");
			return;
		}
	}

	if (sdcard_read(block, count, dst) != SD_OK) {
		printf("Error: SDCard read failed\n");
		return;
	}
	/* Only dump single-block reads (multi-block reads are memory loads) */
	if (count == 1)
		dump_bytes((unsigned int *)dst, 512, (unsigned long)dst);
}

define_command(sdcard_read, sdcard_read_handler, "Read SDCard block", LITESDCARD_CMDS);
#endif

/**
 * Command "sdcard_write"
 *
 * Perform SDCard block write
 *
 */
#ifdef CSR_SDCARD_MEM2BLOCK_DMA_BASE_ADDR
static void sdcard_write_handler(int nb_params, char **params)
{
	int i;
	uint8_t buf[512];
	unsigned int block;
	char *c;

	if (nb_params < 2) {
		printf("sdcard_write <block> <str>\n");
		return;
	}

	block = strtoul(params[0], &c, 0);
	if (*c != 0) {
		printf("Error: invalid block number\n");
		return;
	}

	c = params[1];
	for(i=0; i<512; i++) {
		buf[i] = *c;
		if(*(++c) == 0) {
			c = params[1];
		}
	}
	dump_bytes((unsigned int *)buf, 512, (unsigned long) buf);
	if (sdcard_write(block, 1, buf) != SD_OK)
		printf("Error: SDCard write failed\n");
}

define_command(sdcard_write, sdcard_write_handler, "Write SDCard block", LITESDCARD_CMDS);
#endif
