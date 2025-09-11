/*------------------------------------------------------------------------/
/  4-bit SD interface mode (not SPI) SD card driver plug-in
/-------------------------------------------------------------------------/
/
/  Copyright (C) 2024,2025, Evan Hillas, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------/

  Features and Limitations:

  * Requires compiling with FlexC v7.0 or newer.

  * Platform Specific - Prop2 - uses hardware DMA resources

  * Uses 6, 7 or 8 I/O pins, 4-bit mode only
    DAT pins must be ordered on a 4-bit pin boundary
      Eg: Pins P0..3 or P4..7 or P8..11 and so on

  * No Media Change Detection
    Application program needs to perform a f_umount()/f_mount() after media change.


Revisions:
Work began end of June 2023 - https://forums.parallax.com/discussion/comment/1551799/#Comment_1551799
10 Oct 2024: v0.1  https://forums.parallax.com/discussion/comment/1562218/#Comment_1562218
14 Oct 2024: v0.2, added SDSC support for old 2 GB cards - https://forums.parallax.com/discussion/comment/1562391/#Comment_1562391
28 Oct 2024: v0.3  https://forums.parallax.com/discussion/comment/1562889/#Comment_1562889
25 Nov 2024: v0.5  https://forums.parallax.com/discussion/comment/1563618/#Comment_1563618
15 Dec 2024: v0.7
 9 Jan 2025: v0.8  https://forums.parallax.com/discussion/comment/1564719/#Comment_1564719
14 Feb 2025: v0.9
17 Feb 2025: v1.0, Released, added to Obex database - https://obex.parallax.com/obex/sdsd-cc/

26 Feb 2025: v1.1, bug fix for multi-card use - Convert DMA parameter structs to non-static
20 Apr 2025: v1.2, bug fix for sharing amongst cogs - Ensure DIR/OUT, for used pins, finish as set low, https://forums.parallax.com/discussion/comment/1566465/#Comment_1566465
26 Apr 2025: v1.3, rework disk_read()/disk_write() to implement lazy CMD12 for sequential performance
30 Apr 2025: v1.4, some size optimising, remove legacy driver interface, bug fix SETSE1 for lazy CMD12
18 May 2025: v1.5, now supports optional FF_USE_TRIM compile switch - Effective when SD card supports erase-discard
26 July 2025: v1.6, fixed a harmless typo that appeared in v1.4: unconditional ROLBYTE instruction in tx block CRC calc
11 Sept 2025: v1.7, changed power down error into just a warning and removed the subsequent excessive power up check

/-------------------------------------------------------------------------*/
#define _SDSD_VERSION_ "v1.7"


#ifndef __propeller2__
#error ONLY SUPPORTS THE PROPELLER 2 MICROCONTROLLER
#endif
#include <propeller2.h>

#include <filesys/fatfs/diskio.h>    // Common include file for FatFs and disk I/O layer
#include <stdlib.h>    // needed for some data types
#include <string.h>    // needed for memset()



#ifdef _DEBUG
    #define SD_DEBUG
#endif
//#define SD_DEBUG_ACCESSES


enum {
    RX_SCHMITT = 0,    // 0 = CMOS threshold (CLK pin is Schmitt), 1 = Schmitt trigger of CMD and DAT pins
//    CMDAT_REG = 0,    // 0 = unregistered pins, 1 = registered CMD and DAT pins
    CLK_REG = 0,    // 0 = unregistered pin, 1 = registered CLK pin
//    CLK_POL = 1,    // 0 = idle low (selects High-Speed), 1 = idle high (selects Default-Speed)
    CLK_DIV_DEFAULT = 4,    // Default to 4 for symetrical clock and also less overclocked beyond spec'd 50 MHz

    // "ledpin" masks, the actual pin number is stored in bits 8..15
    BYTE_ADDRESSING_MASK = 1,
    BLOCKREAD_CRC_MASK = 2,
    CLK_POLARITY_MASK = 4,
    CARD_CACHING_MASK = 8,
    TRIM_DISCARD_MASK = 16,

//--------------------------------------------------------------------------
// driver specific ioctl() commands
//--------------------------------------------------------------------------
    CTRL_BLOCKREAD_CRC = 70,    // Disables/enables CRC processing of block read, default is enabled
    CTRL_GET_CLKDIV = 71,    // Get the active clock-divider
    CTRL_SET_CLKDIV = 72,    // Set the active clock-divider
    CTRL_GET_CLKPOL = 73,    // (Not implemented)
    CTRL_SET_CLKPOL = 74,    // (Not implemented)
};




// SD Card/Socket specific variables
uint8_t  cidbytes[18];
uint16_t  rca16;    // active card's 16-bit RCA register
LBA_t  discblocks;    // active card's user capacity
uint16_t  clkdivider;    // full speed clock-divider, minimum value of 2
uint8_t  clkpin;  // pin number of clock pin
uint8_t  cmdpin;  // pin number of command/response pin
uint8_t  dat0pin;  // base pin number of the four DAT pins
uint8_t  pwrpin;  // pin number of power switch (-1 == not present)
uint16_t ledpin;  // pin number of activity LED (-1 == not present), in upper 8 bits
                  // doubles up as SDSC byte addressing flag in bit0, autodetected
                  // and block read CRC enable flag in bit1, defaults to set, overridden by IOCTL
                  // and clock polarity flag in bit2, autodetected
                  // and SDHC caching flag in bit3, autodetected, (feature removed, no benefit without queuing)
                  // and Erase Discard flag in bit4, autodetected
uint8_t rxlagcomp;  // Compensates for cumulative clk-rx latencies, block read errors trigger a recalibration



//-----------------------------------------------------------------------
// Orderly clean up of pin and smartpin modes
//-----------------------------------------------------------------------

static void  releasepins( void )
{
    unsigned  PIN_CLK = clkpin;
    unsigned  PIN_CMD = cmdpin;
    unsigned  PIN_DAT = dat0pin | 3<<6;
    unsigned  PIN_PWR = pwrpin;
    unsigned  PIN_LED = ledpin >> 8;    // Doubles up as SDSC byte addressing flag in bit0

    _pinf(PIN_CLK);    // stop SD clock

    _pinf(PIN_CMD);
    _pinf(PIN_DAT);
    _wrpin(PIN_CMD, 0);
    _wrpin(PIN_DAT, 0);

    _waitus(1);    // wait for pull-up resistors to act

    _wrpin(PIN_CLK, 0);    // release clock pin

    if( PIN_LED != PIN_CLK ) {    // activity LED is present
        _pinf(PIN_LED);
        _wrpin(PIN_LED, 0);
    }
    if( PIN_PWR != PIN_CLK ) {    // power switch is present
        _pinf(PIN_PWR);
        _wrpin(PIN_PWR, 0);
    }

    _waitus(1);    // wait for pull-up to act
}



//-----------------------------------------------------------------------
// Give up control of pins by this cog
//-----------------------------------------------------------------------

static void  give_pins( void )
{
    _fltl(clkpin);
    _fltl(dat0pin | 3<<6);
    _fltl(cmdpin);
}



//-----------------------------------------------------------------------
// Take control of clock pin - used by block r/w jointing feature
//-----------------------------------------------------------------------

static void  take_clkpin( void )
{
    uint32_t  PIN_CLK = clkpin;

    __asm {
//		setse1	#0    // cancel triggering before reuse - not needed if POLLSE1 is used
		drvl	PIN_CLK    // enable CLK smartpin
		or	PIN_CLK, #0b001<<6;    // trigger on rising edge - clocks completed
		setse1	PIN_CLK
    }
}



//-----------------------------------------------------------------------
// Wait for card ready - Checks the DAT0 pin write-data-busy indicator
//-----------------------------------------------------------------------

static int  wait_ready(    // 0:Card Busy, 1:Card Ready
    uint32_t timeout )    // 100/250/500 ms interval in sysclock ticks
{
    unsigned  PIN_CLK = clkpin;
    uint32_t  m_se2;
    int  ready;

    _drvl(PIN_CLK);    // enable CLK smartpin
    _wypin(PIN_CLK, -1);
    timeout += _cnt();
    m_se2 = 0b110<<6 | dat0pin;    // trigger on high level - card ready
    ready = 0;   // Busy

    __asm {    // "const" enforces XIP, "volatile" enforces Fcache
		setse2	#0    // cancel triggering before reuse - not needed if POLLSE2 is used
		setse2	m_se2
		setq	timeout
		waitse2   wc
	if_nc	mov	ready, #1    // Not busy
    }

#ifdef SD_DEBUG
    if( !ready )
        __builtin_printf(" Busy timeout! ");
#endif

    return ready;
}



//----------------------------------------------------------------------------
// Compute the CRC7 of some bytes, matched to framing of SD command-response
//----------------------------------------------------------------------------

static uint32_t  crc7sd(    // SD spec 4.5
    uint8_t *buf,
    size_t len )
{
    uint32_t  crc = 0;

    __asm volatile {    // "const" enforces XIP, "volatile" enforces Fcache
// Reference code courtesy of Ariba
		rdfast	#0, buf
		rep	@.rend, len
		rfbyte	pa
		movbyts	pa, #0b00_01_10_11    // byte swap within longwords
		setq	pa
		crcnib	crc, #0x48    // CRC-7-ITU reversed (x7 + x3 + x0: 0x09, odd parity)
		crcnib	crc, #0x48
.rend
		rev	crc    // correct the bit order to match standard
		shr	crc, #24    // 7-bit CRC in bits 7..1
		or	crc, #1    // and the SD response end-bit in bit0
    }
    return crc;
}



//-----------------------------------------------------------------------
// Extract disc capacity from the CSD bits, both v1.0 and v2.0 variants
//-----------------------------------------------------------------------

static LBA_t  disc_size(
    uint8_t *csd )
{
    uint32_t  cs = __builtin_bswap32(*(uint32_t *)&csd[6]);
    int  n;

    if( csd[0]>>6 ) {    // SDC ver 2.00
        cs = (cs & 0xfff_ffff) + 1;  // 0.5 MiByte granularity
        n = 10;
    } else {    // SDC ver 1.00
        cs = (cs>>14 & 0xfff) + 1;    // C_SIZE
        n = (__builtin_bswap16(*(uint16_t *)&csd[9])>>7 & 0x7) + 2    // C_SIZE_MULT
            + (csd[5] & 15) - 9;    // READ_BL_LEN
    }

    return (LBA_t)cs << n;    // 32/64-bit block count
}



//-----------------------------------------------------------------------
// Robust power cycling of the SD Card, optional - if pin defined
//-----------------------------------------------------------------------

static int  sdcard_power( void )
{
    unsigned  PIN_VOLT = clkpin;
    unsigned  PIN_CMD = cmdpin;
    unsigned  PIN_DAT = dat0pin | 3<<6;
    unsigned  PIN_PWR = pwrpin;    // pin number of power switch (-1 == not defined)
    uint32_t  samples, threshold;
    int32_t  tmr, timeout;

    releasepins();    // SD spec 6.4.2

    _pinl(PIN_CMD);
    _pinl(PIN_DAT);
    _waitus(1);
    _pinf(PIN_CMD);
    _pinf(PIN_DAT);
    _waitus(1);    // wait for pull-up resistors to act

    if( PIN_PWR == PIN_VOLT ) {    // power switch not present
        if( _pinr(PIN_CMD) )
            if( _pinread(PIN_DAT) == 15 )
                return 1;    // pull-up resistors are present
#ifdef SD_DEBUG
        __builtin_printf(" Error: CMD/DAT pull-up resistors are missing\n");
#endif
        return 0;
    }

    if( _pinr(PIN_PWR) ) {    // An inserted card will pull this down against the 22k ohm pull-up
#ifdef SD_DEBUG
        __builtin_printf(" Error: SD Card not detected\n");
#endif
        return 0;
    }
#ifdef SD_DEBUG
    __builtin_printf(" Card detected ... power cycle of SD card\n");
#endif

// Use CompDAC pin mode to wait for supply to fall below 0.5 Volt, SD spec 6.4.1.2
    threshold = (uint32_t)(0.5 / 3.4 * 255.0);    // 0.5 Volts
    timeout = 500 * 32;    // 500 ms
#ifdef SD_DEBUG
    __builtin_printf("  power-down threshold = %d   pin state = %d\n", threshold, _pinr(PIN_VOLT));
#endif
    _wrpin(PIN_VOLT, P_LEVEL_A | threshold<<8);    // set CompDAC threshold to 0.5 volts
    _pinh(PIN_PWR);    // power off the SD card by overpowering the 1800 ohm Card Detect
    tmr = _cnt();
    _pinl(PIN_DAT);    // drive DAT pin group low to discharge the now unpowered capacitors
    _pinl(PIN_CMD);    // ditto CMD
    _waitus(1);    // DAC settle time is about 1.0 us
    samples = _pinr(PIN_VOLT);
    do {
        _waitus(32);    // 32 us x 32 samples = 1.0 ms
        samples = samples<<1 | _pinr(PIN_VOLT);
        timeout--;
    } while( samples && timeout );    // unanimous wait for pin to drop below threshold level
    tmr = _cnt() - tmr;

#ifdef SD_DEBUG
    if( samples )
        __builtin_printf("WARNING: Stuck high at SD power-down\n");

    tmr = _muldiv64(tmr, 1_000_000, _clockfreq());
    __builtin_printf("  power-down slope = %d us   pin state = %d\n", tmr, _pinr(PIN_VOLT));
#endif

    releasepins();    // and power back on
    _waitms(2);    // delay for card supply ramp up plus card start up, SD spec 6.4.1.1

    return 1;
}



//-----------------------------------------------------------------------
// Transmit a data block to the SD card
//-----------------------------------------------------------------------
// Return codes:
//  %10_010_1 == CRC matched
//  %10_101_1 == CRC mismatch
//  %11_111_1 == CRC no response
//  %00_000_0 == card busy timeout

static int  tx_datablock(
    const void *buf,    // Buffer address of block data to be transmitted
    uint32_t timeout )    // 250 ms timeout for card busy, in sysclock ticks
    __attribute__(opt(no-fast-inline-asm))    // use the full assembler, for RES fitting, duplicates C locals
{
    void  *ptr = &txblkset;

    __asm volatile {    // "const" enforces XIP, "volatile" enforces Fcache
// retrieve parameter set
		setq	#sizeof(txblkset) / 4 - 1
		rdlong	p_clk, ptr    // fast copy to cogRAM
// start clocking to see when card exits from write busy status
		wypin	lnco, p_clk    // busy check
		getct	pb    // timeout begins
//		testb	timeout, #0   wc    // diag to force a CRC error
		add	timeout, pb    // SDHC 250 ms timeout of block erase-write (SD spec 4.6.2.2)

		setse2	m_se2    // trigger on DAT0 high level - card ready
// preload block data into CogRAM for CRC processing
		setq	#512/4-1    // one SD data block
		rdlong	cogdatbuf, buf    // fast copy to cogRAM before engaging the FIFO

		pollse1       // clear stale event
		pollse2       // clear stale event
		rdfast	lnco, buf    // start the FIFO, non-blocking init
		setxfrq	lnco    // sysclock/1 for lead-in timing
		setq	timeout    // apply the 250 ms timeout to WAITSEn
		waitse2   wz    // wait for ready (DAT0 high), Z set if timed-out - still busy

		wypin	#2, p_clk    // minimum turnaround, Nwr (SD spec 4.12.4)
		waitse1    // wait for clocking to complete

		dirl	p_clk    // reset smartpin
	if_nz	drvl	p_dat    // start-bit placed on data pins

	if_nz	xinit	m_align, #0    // lead-in delay from here at sysclock/1
	if_nz	setq	v_nco        // streamer transfer rate (takes effect with buffered command below)
	if_nz	xzero	m_dat, #0     // buffered-op, aligned to clock via lead-in
		dirh	p_clk    // clock timing starts here
	if_nz	wypin	clocks, p_clk    // first pulse outputs during second clock period

// compute the SD data block's CRC
	if_nz	rep	@.rend1, #512/4    // one SD data block
	if_nz	alti	altireg, #0b100_111_000    // next D-field substitution, then increment altireg, result goes to PA
	if_nz	movbyts	pa, #0b00_01_10_11    // byte swap within longword
	if_nz	splitb	pa    // every 4th bit makes a byte, 8 parallel to 4 serial
	if_nz	setq	pa
	if_nz	crcnib	crc3, poly
	if_nz	crcnib	crc3, poly
	if_nz	crcnib	crc2, poly
	if_nz	crcnib	crc2, poly
	if_nz	crcnib	crc1, poly
	if_nz	crcnib	crc1, poly
	if_nz	crcnib	crc0, poly
	if_nz	crcnib	crc0, poly
.rend1
	if_nz	rolbyte	pa, crc3, #0
	if_nz	rolbyte	pa, crc2, #0
	if_nz	rolbyte	pa, crc1, #0
	if_nz	rolbyte	pa, crc0, #0
	if_nz	mergeb	pa    // every 8th bit makes a nibble, 4 serial to 8 parallel

	if_nz	waitse1    // wait for clocking to complete

// transmitt the CRC
	if_nz	setxfrq	lnco    // sysclock/1 for lead-in timing
	if_nz	dirl	p_clk    // reset smartpin
	if_nz	xinit	m_align2, #0    // lead-in delay from here at sysclock/1
	if_nz	setq	v_nco        // streamer transfer rate (takes effect with buffered command below)
	if_nz	xzero	m_crc, pa    // 8 nibbles, transmit low nibble first, suits the reversed CRC
	if_nz	dirh	p_clk    // clock timing starts here
	if_nz	wypin	v_crc, p_clk    // 16-bit CRC + end-bit (either 18 or 19 clocks)

	if_nz	rolbyte	pa, crc3, #1
	if_nz	rolbyte	pa, crc2, #1
	if_nz	rolbyte	pa, crc1, #1
	if_nz	rolbyte	pa, crc0, #1
//if_nc_and_nz	rolbyte	pa, crc0, #1    // diag, modified to force an error
	if_nz	mergeb	pa    // every 8th bit makes a nibble, 4 serial to 8 parallel
	if_nz	xzero	m_crc, pa    // 8 nibbles, transmit low nibble first, suits the reversed CRC
	if_nz	xzero	m_crc, endbit
	if_nz	waitse1    // wait for clocking to complete

// tx complete, now rx "CRC status"
		dirl	p_dat    // release DAT pins ready for CRC status from SD card
		and	p_dat, #0x3f    // strip away ADDPINS, leaving DAT0

		rep	@.rend2, #6
		wypin	#1, p_clk    // one clock pulse
		testp	p_dat   wc    // sample DAT0 before the clock pin transitions
		rcl	timeout, #1    // collect status bits
		waitse1      // wait for each clock done
.rend2
	_ret_	wypin	#500, p_clk    // trailing clocks
//		ret    // RET not needed, full assembler can handle _RET_ instead

clocks		long 1 + 512 * 2    // start-bit + nibble count
lnco		long 0x8000_0000    // lead-in rate, sysclock/1
endbit		long 0xffff_ffff
poly		long 0x8408    // CRC-16-CCITT reversed (x16 + x12 + x5 + x0: 0x1021, even parity)
altireg		long pa<<19 | cogdatbuf<<9    // register PA for ALTI result substitution, and CRC buffer address
crc3		long 0
crc2		long 0
crc1		long 0
crc0		long 0

// copy of txblkset parameters
p_clk		res 1    // CLK pin number
p_dat		res 1    // DAT pin number + ADDPINS
m_align		res 1    // Streamer mode word, data leadin timing
v_nco		res 1    // NCO divider, streamer transfer rate
m_se2		res 1    // SETSET2 mode word
m_dat		res 1    // Streamer mode word, data timing
m_crc		res 1    // Streamer mode word, CRC timing
m_align2	res 1    // Streamer mode word, CRC leadin timing
v_crc		res 1    // CRC clocks, compensates CRC status timing on clock polarity change

cogdatbuf	res 512/4    // longwords for data block
    }

    timeout &= 0x3f;    // remove excess bits from "CRC status"

#ifdef SD_DEBUG
    if( timeout != 0b10_010_1 ) {    // CRC didn't match
        __builtin_printf(" BlockWriteError: %%%06b ", timeout);
        if( timeout == 0 )
            __builtin_printf("Data busy timeout!\n");
        else if( timeout == 0b10_101_1 )
            __builtin_printf("CRC mismatched!\n");
        else
            __builtin_printf("Unknown CRC Status\n");
    }
#endif

    return timeout;  // $00 = busy timeout, $3F = CRC no response, %10_101_1 = CRC mismatch, %10_010_1 = CRC matched
}


//-----------------------------------------------------------------------
// Tx data block's DMA parameter setup
//-----------------------------------------------------------------------

struct txblk_parms_t {
    uint32_t  p_clk, p_dat, m_align, v_nco, m_se2, m_dat, m_crc, m_align2, v_crc;
} txblkset;


static void  set_txblkset(
    uint32_t clkdiv,
    uint32_t CMDAT_REG,
    uint32_t CLK_POL )
{
    uint32_t  PIN_DAT0 = dat0pin;
    uint32_t  poldiv = (CLK_POL ? clkdiv : clkdiv/2);
    CMDAT_REG = CMDAT_REG & 1;    // extract lsb from rxlag

    txblkset.p_clk = clkpin;
    txblkset.p_dat = PIN_DAT0 | 3<<6;    // DAT0..3 ordered pin group
    txblkset.m_se2 = PIN_DAT0 | 0b110<<6;    // trigger on high level - card ready
    txblkset.m_align = X_IMM_32X1_1DAC1 | 5 + CLK_REG - CMDAT_REG + clkdiv + poldiv;  // start(S)-bit is ahead of first streamer bit
    txblkset.m_align2 = X_IMM_32X1_1DAC1 | 5 + CLK_REG - CMDAT_REG + poldiv;    // no S-bit, CRC abuts end of data block
    txblkset.v_nco = 0x8000_0000UL / clkdiv + (0x8000_0000UL % clkdiv > 0 ? 1 : 0);  // data transfer rate, round up upon a non-zero remainder
    txblkset.m_dat = X_RFBYTE_4P_1DAC4 | PIN_DAT0<<17 | X_PINS_ON | X_ALT_ON | 512 * 2;    // mode and nibble count, msnib first
    txblkset.m_crc = X_IMM_8X4_1DAC4 | PIN_DAT0<<17 | X_PINS_ON | 8;    // mode and nibble count, lsnib first
    txblkset.v_crc = 16 + 2 + (CLK_POL ? 1 : 0);    // CRC clocks, 18 or 19
}



//-----------------------------------------------------------------------
// Receive data blocks from the SD card
//-----------------------------------------------------------------------

static int  rx_datablocks(    // 0 = success, 1 = CRC fail, 2 = start-bit timeout
    void *buf,    // Buffer address to store the received data blocks
    uint32_t blocks,    // Number of data blocks to be read
    uint32_t timeout,    // 100 ms timeout for each start-bit latency, in sysclock ticks
    uint8_t *crcbuf )    // extra hubRAM for holding the final 16 CRC nibbles
    __attribute__(opt(no-fast-inline-asm))    // use the full assembler, for RES fitting, duplicates C locals
{
    void  *ptr = &rxblkset;

    __asm volatile {    // "const" enforces XIP, "volatile" enforces Fcache
		setq	#sizeof(rxblkset) / 4 - 1
		rdlong	p_clk, ptr    // fast copy to cogRAM

		fltl	p_dat
		setse2	m_se2    // trigger on DAT0 low level, preferred to falling edge trigger
		// because DAT pins idle high during command and response and also ensures best chance
		// of seeing an early start-bit
		modz	_clr   wz    // clear Z flag to create a one-shot for first block
		pollse1
// SKIPF patterns:
//   (a)  Ignore CRC, first block
//   (b)  Ignore CRC, further blocks
//   (c)  Process CRC, except last block
//   (d)  Ignore CRC, last block
//   (e)  Process CRC, last block
nextblk
		wrfast	lnco, buf    // setup FIFO for streamer use                                             a |   c

		skipf	skip1    // a/b = Ignore CRC, c = Process CRC                                           +++++++
		wxpin	v_sdiv, p_clk    // clock-divider for start-bit search                                  a b   c
// no room in the main buffer for final CRC-16 nibbles - to be handled later
		cmp	blocks, #2   wc    // last block?                                                       | |   c
	if_c	sub	clocks, #16+2    // stop the clocks before the CRC-16 nibbles                           | |   c
	if_c	sub	m_dat, #16    // truncate DMA to end of buffer                                          | |   c
// locate SD data block start-bit
// NOTE: Search code has a one SD clock lag.  The start(S)-bit has been and gone by the time it triggers.
//   The following streamer setup begins its sampling ahead of the second data bit arriving.
		wypin	lnco, p_clk    // start the search                                                      a b   c
		getct	pb    // timeout begins                                                                 a b   c
		pollse2       // clear stale event                                                              a b   c
		setxfrq	lnco    // set streamer to sysclock/1 for lead-in timing                                a b   c
		add	pb, timeout    // SDHC 100 ms timeout of start-bit search, Nac (SD spec 4.6.2.1)        a b   c
		setq	pb    // apply the 100 ms timeout to WAITSEn                                            a b   c
		waitse2   wc    // wait for DAT0 low, sets C upon time-out                                      a b   c

// start-bit found
		dirl	p_clk    // halt clock-gen ASAP                                                         a b   c
		wxpin	v_div, p_clk    // clock-divider to match streamer rate                                 a b   c

// now read the SD data block
	if_nc	xinit	m_align, #0    // lead-in delay from here at sysclock/1                                 a b   c
	if_nc	setq	v_nco    // streamer transfer rate (takes effect with buffered command below)           a b   c
	if_nc	xzero	m_dat, #0     // buffered-op, aligned to clock via lead-in                              a b   c
		dirh	p_clk    // clock timing starts here                                                    a b   c
	if_nc	wypin	clocks, p_clk    // first pulse outputs during second clock period                      a b   c

	if_nc	call	#crc_check    // returned with Z set is CRC passed, Z is clear on entry for first block | |   c

	if_nc	waitse1    // wait for clocking to complete, fresh block received                               a b   c
		setq	#512/4+16/8-1    // one data block + CRC, copy the fresh block                          | |   c
		rdlong	cogdatbuf, buf    // fast copy to cogRAM before engaging the FIFO                       | |   c

	if_nc	add	buf, blksize    // prep for next block                                                  | |   c
	if_nc	djnz	blocks, #nextblk    //                                                                  a b   |
if_nc_and_z	djnz	blocks, #nextblk    //                                                                  | |   c

// Now collect final data block's CRC nibbles.
// Since there is no room left in the main data buffer, this needs
// aditional code to restart the streamer at a separate buffer
		skipf	skip2    // d = Ignore CRC, e = Process CRC                                               +++++
	if_c	mov	blocks, #2    // indicate a start-bit time-out                                            d   e

if_nc_and_z	wrfast	lnco, crcbuf    // setup FIFO for streamer use, assumes mdat holds stale streamer mode    |   e
if_nc_and_z	setxfrq	lnco    // set streamer to sysclock/1 for lead-in timing                                  |   e
if_nc_and_z	dirl	p_clk    // reset clock smartpin                                                          |   e

if_nc_and_z	xinit	m_align, #0    // lead-in delay from here at sysclock/1                                   |   e
if_nc_and_z	setq	v_nco    // streamer transfer rate (takes effect with buffered command below)             |   e
if_nc_and_z	xzero	m_crc, #0    // buffered-op, aligned to clock via lead-in                                 |   e
if_nc_and_z	dirh	p_clk    // clock timing starts here                                                      |   e
if_nc_and_z	wypin	#16+2, p_clk    // CRC-16 nibbles + end bit                                               |   e

if_nc_and_z	waitse1    // wait for clocking to complete                                                       |   e

// copy CRC nibbles to cogRAM then process
if_nc_and_z	setq	#16/8-1    // 16 CRC-16 nibbles, 8 nibbles per longword                                   |   e
if_nc_and_z	rdlong	cogcrcbuf, crcbuf    //                                                                   |   e
if_nc_and_z	call	#crc_check    //                                                                          |   e

if_nc_and_nz	mov	blocks, #1    // indicate a failed CRC                                                    |   e
		ret

crc_check
// CRC-16 check of prior read data block
// Z is clear upon first data block of transaction, returns quickly with Z set
		mov	crc3, #0
		mov	crc2, #0
		mov	crc1, #0
		mov	crc0, #0
		mov	pb, altireg
	if_z	rep	@.rend, #512/4+16/8    // one SD data block + CRC

	if_z	alti	pb, #0b100_111_000    // next D-field substitution, then increment PB, result goes to PA
	if_z	movbyts	pa, #0b00_01_10_11    // byte swap within longword
	if_z	splitb	pa    // every 4th bit makes a byte, 8 parallel to 4 serial
	if_z	setq	pa
	if_z	crcnib	crc3, poly
	if_z	crcnib	crc3, poly
	if_z	crcnib	crc2, poly
	if_z	crcnib	crc2, poly
	if_z	crcnib	crc1, poly
	if_z	crcnib	crc1, poly
	if_z	crcnib	crc0, poly
	if_z	crcnib	crc0, poly
.rend
// pass/fail
		or	crc3, crc2
		or	crc1, crc0
	_ret_	or	crc3, crc1   wz    // Z set for pass, clear for fail
//		ret    // full assembler can handle _RET_ instead

clocks		long 512 * 2 + 16 + 2    // clock pulses, nibble count + 16-bit CRC + end-bit
lnco		long 0x8000_0000    // lead-in rate, sysclock/1
poly		long 0x8408    // CRC-16-CCITT reversed (x16 + x12 + x5 + x0: 0x1021, even parity)
altireg		long pa<<19 | cogdatbuf<<9    // register PA for ALTI result substitution, and CRC buffer address
blksize		long 512

// copy of rxblkset parameters
p_clk		res 1    // CLK pin number
p_dat		res 1    // DAT pin number + ADDPINS
m_align		res 1    // Streamer mode word, data leadin timing
v_nco		res 1    // NCO divider, streamer transfer rate
m_se2		res 1    // SETSET2 mode word
m_dat		res 1    // Streamer mode word, for data timing
m_crc		res 1    // Streamer mode word, for CRC timing
v_div		res 1    // Clock divider, data rate, post-v_sdiv reset
v_sdiv		res 1    // Clock divider, start-bit search rate
skip1		res 1    // SKIPF pattern, ignore CRC or process CRC
skip2		res 1    // SKIPF pattern, final block CRC

crc3		res 1
crc2		res 1
crc1		res 1
crc0		res 1

cogdatbuf	res 512/4    // longwords for SD data block
cogcrcbuf	res 16/8    // longwords for CRC nibbles, must be contiguous with the SD data block
    }

#ifdef SD_DEBUG
    if( blocks ) {
        __builtin_printf(" BlockReadError:  err=%d  ", blocks);
        if( blocks == 2 )
            __builtin_printf("start-bit timed out!\n");
        else
            __builtin_printf("CRC failed!\n");
    }
#endif

    return blocks;    // 0 = success, 1 = CRC fail, 2 = start-bit timeout
}


//-----------------------------------------------------------------------
// Rx data block's DMA parameter setup
//-----------------------------------------------------------------------

struct rxblk_parms_t {
    uint32_t  p_clk, p_dat, m_align, v_nco, m_se2, m_dat, m_crc, v_div, v_sdiv, skip1, skip2;
} rxblkset;


static void  set_rxblkcrc( void )
{
    rxblkset.m_dat &= 0xffff_0000;    // remove existing nibble count

    if( ledpin & BLOCKREAD_CRC_MASK )  {    // exec path includes CRC processing
        rxblkset.m_dat |=  512 * 2 + 16;  // apply nibble count, 16 CRC nibbles
        rxblkset.skip1 = 0b00000000_10000000_00000000_00000000;    // pat-c
        rxblkset.skip2 = 0b00000000_00000000_00000000_00000000;    // pat-e
    } else {    // exec path excludes CRC processing
        rxblkset.m_dat |= 512 * 2;  // apply nibble count
        rxblkset.skip1 = 0b00000001_01110100_00000000_00001110;    // pat-a and pat-b
        rxblkset.skip2 = 0b00000000_00000000_00111111_11111110;    // pat-d
    }
}


static void  set_rxblkset(
    uint32_t clkdiv,
    uint32_t rxlag )
{
    uint32_t  PIN_DAT0 = dat0pin;

    if( rxlag & 1 )    // pins registered
        rxlag += 2;    // linear correction, registered pins take one tick longer than unregistered
    rxlag >>= 1;    // latency component

    rxblkset.p_clk = clkpin;
    rxblkset.p_dat = PIN_DAT0 | 3<<6;
    rxblkset.m_se2 = PIN_DAT0 | 0b100<<6;    // trigger on low level, preferred to falling edge trigger because
        // DAT pins idle high during command and response and also ensures best chance of seeing an early start-bit
    if( clkdiv < 20 )  {    // start-bit search ends on first data
        rxblkset.v_sdiv = 10 | 5<<16;    // search clock-divider
        rxblkset.m_align = X_IMM_32X1_1DAC1 | 8 + clkdiv + rxlag;    // skip over clock startup only
    } else {    // start-bit search ends on start(S)-bit
        rxblkset.v_sdiv = clkdiv | clkdiv/2<<16;    // search clock-divider
        rxblkset.m_align = X_IMM_32X1_1DAC1 | 8 + clkdiv*2 + rxlag;    // skip over clock startup plus the S-bit
    }
    rxblkset.v_div = clkdiv | clkdiv/2<<16;
    rxblkset.v_nco = 0x8000_0000UL / clkdiv + (0x8000_0000UL % clkdiv > 0 ? 1 : 0);  // data transfer rate, round up upon a non-zero remainder

    rxblkset.m_dat = X_4P_1DAC4_WFBYTE | PIN_DAT0<<17 | X_PINS_ON | X_ALT_ON;  // mode component only, msnib first
    rxblkset.m_crc = X_4P_1DAC4_WFBYTE | PIN_DAT0<<17 | X_PINS_ON | X_ALT_ON | 16;  // mode and nibble count, msnib first

    set_rxblkcrc();    // set the skip patterns and nibble count for selected exec path
}



//-----------------------------------------------------------------------
// Receive the SD card's command-response packet
//-----------------------------------------------------------------------

static int  rx_response(    // 0:Start-bit timed out
    uint8_t *resp,    // Buffer address for response bytes
    uint32_t len,    // Number of bytes in response, including the start-end framing bits
    uint32_t clocks )    // Additional clocks for card's post-response idle transition
{
    void *ptr = &crset;    // needed when non-static structs used - which is required for instancing

    __asm volatile {    // "const" enforces XIP, "volatile" enforces Fcache
//		loc	pb, #crset    // static struct required
		setq	#sizeof(crset) / 4 - 1
		rdlong	p_clk, ptr    // fast copy to cogRAM

		shl	len, #3
		setword	m_resp, len, #0    // add bit count to DMA mode word, crset.m_resp
		add	clocks, len
//		setse2	#0    // cancel triggering before reuse - not needed if POLLSE2 is used
		setse2	m_se2    // trigger on falling edge of CMD pin - start-bit arrived
		waitse1    // wait for end of command tx clocks

// locate response start-bit
// NOTE: Search code has a one SD clock lag.  The start(S)-bit has been and gone by the time it triggers.
//   The following streamer setup samples the T bit twice to fill the S bit in the received bits.
		outh	p_cmd    // when driven, momentarily force CMD pin high before start-bit search
		wxpin	v_sdiv, p_clk    // clock-divider for start-bit search
		wypin	lnco, p_clk    // start the search
		fltl	p_cmd    // pin drive release before SD card takes over
		pollse2       // clear stale event
		wrfast	lnco, resp    // engage the FIFO, non-blocking init
		setxfrq	lnco    // sysclock/1 for lead-in timing
		getct	pb    // timeout begins
		add	pb, timeout    // max of 64 clocks for response to start, Ncr (SD spec 4.12.4)
		setq	pb    // apply the 64 clock timeout to WAITSEn
		waitse2   wc    // wait for CMD start bit, C set if timed-out

// start-bit found
		dirl	p_clk    // halt clock-gen ASAP
		wxpin	v_div, p_clk    // clock-divider to match streamer rate

// now read the response (NOTE: start-bit is part of first byte)
		xinit	m_align, #0    // lead-in delay from here at sysclock/1
	if_nc	setq	v_nco    // streamer transfer rate (takes effect with buffered command below)
	if_nc	xzero	m_resp, #0     // buffered-op aligned to clock via lead-in
		dirh	p_clk    // clock timing starts here
	if_nc	wypin	clocks, p_clk    // first clock pulse outputs during second clock period

		// twiddle thumbs
	if_c	mov	len, #0    // start-bit timed out

		waitxfi      // wait for rx data completion before resuming hubexec
		ret    // mini assembler requires a standalone RET

lnco		long 0x8000_0000    // lead-in rate, sysclock/1

// copy of crset parameters
p_clk		res 1    // CLK pin number
p_cmd		res 1    // CMD pin number
m_align		res 1    // Streamer mode word, response leadin timing
v_nco		res 1    // NCO divider, streamer transfer rate
m_se2		res 1    // SETSET2 mode word
timeout		res 1    // Start-bit search timeout, in sysclock ticks
m_resp		res 1    // Streamer mode word, response data timing
v_div		res 1    // Clock divider, data rate, post-v_sdiv reset
v_sdiv		res 1    // Clock divider, start-bit search rate
    }
#ifdef SD_DEBUG
    if( !len )
        __builtin_printf(" Response start-bit timeout! ");
#endif

    return len;    // 0:Start-bit timed out
}


//-----------------------------------------------------------------------
// Command response's DMA parameter setup
//-----------------------------------------------------------------------

struct cmdresp_parms_t {
    uint32_t  p_clk, p_cmd, m_align, v_nco, m_se2, timeout, m_resp, v_div, v_sdiv;
} crset;


static void  set_respset(
    uint32_t clkdiv,
    uint32_t rxlag )
{
    uint32_t  PIN_CMD = cmdpin;

    if( rxlag & 1 )    // pins registered
        rxlag += 2;    // linear correction, registered pins take one tick longer than unregistered
    rxlag >>= 1;    // latency component

    crset.p_clk = clkpin;
    crset.p_cmd = PIN_CMD;
    crset.m_se2 = PIN_CMD | 0b010<<6;    // trigger on falling edge of CMD pin - start-bit arrived
    if( clkdiv < 20 )  {    // start-bit search ends on T-bit
        crset.v_sdiv = 10 | 5<<16;    // search clock-divider
        crset.timeout = 10 * 64;    // max of 64 clocks for response to start, Ncr (SD spec 4.12.4)
        crset.m_align = X_IMM_32X1_1DAC1 | 8 + rxlag;    // store T-bit twice to fill both S and T bits in memory
    } else {    // start-bit search ends on start(S)-bit
        crset.v_sdiv = clkdiv | clkdiv/2<<16;    // search clock-divider
        crset.timeout = clkdiv * 64;    // max of 64 clocks for response to start, Ncr (SD spec 4.12.4)
        crset.m_align = X_IMM_32X1_1DAC1 | 8 + clkdiv + rxlag;    // real start(S)-bit is collected
    }
    crset.v_div = clkdiv | clkdiv/2<<16;    // response clock-divider
    crset.v_nco = 0x8000_0000UL / clkdiv + (0x8000_0000UL % clkdiv > 0 ? 1 : 0);  // data transfer rate, round up upon a non-zero remainder
    crset.m_resp = X_1P_1DAC1_WFBYTE | PIN_CMD<<17 | X_PINS_ON | X_ALT_ON;    // + bit count, msbit first
}



//-----------------------------------------------------------------------
// Command's DMA parameter setup
//-----------------------------------------------------------------------

struct cmd_parms_t {
    uint32_t  p_clk, p_cmd, m_align, v_nco, m_ca, m_se1;
} cmdset;

enum {    // address hack to land presets in the Fcache area, tx_command() isn't assigned to Fcache itself
      rp_clk = 0, rp_cmd, rm_align, rv_nco, rm_ca, rm_se1
};


static void  set_cmdset(
    uint32_t clkdiv,
    uint32_t CMDAT_REG,
    uint32_t CLK_POL )
{
    uint32_t  PIN_CMD = cmdpin, PIN_CLK = clkpin;

    CMDAT_REG = CMDAT_REG & 1;    // extract lsb from rxlag

    cmdset.p_clk = PIN_CLK;
    cmdset.p_cmd = PIN_CMD;
    cmdset.m_align = X_IMM_32X1_1DAC1 | 5 + CLK_REG - CMDAT_REG + (CLK_POL ? clkdiv : clkdiv/2);
    cmdset.v_nco = 0x8000_0000UL / clkdiv + (0x8000_0000UL % clkdiv > 0 ? 1 : 0);  // data transfer rate, round up upon a non-zero remainder
    cmdset.m_ca = X_IMM_32X1_1DAC1 | PIN_CMD<<17 | X_PINS_ON | 32;    // mode and bit count, lsbit first
    cmdset.m_se1 = PIN_CLK | 0b001<<6;    // SETSE1 mode, trigger on rising edge - clocks completed
}


//-----------------------------------------------------------------------
// Transmit a command packet to the SD card
//-----------------------------------------------------------------------

static int  tx_command(    // 0:CMD pin stuck low
    uint32_t cmd,    // Command
    uint32_t arg )    // Argument
{
    void *ptr = &cmdset;    // needed when non-static structs - which is required for instancing

    __asm const {    // "const" enforces XIP, "volatile" enforces Fcache
//		loc	pb, #cmdset    // static struct required
		setq	#sizeof(cmdset) / 4 - 1
		rdlong	rp_clk-0, ptr    // fast copy to cogRAM

		drvl	rp_clk-0    // enable CLK smartpin
		setse1	#0    // cancel triggering before reuse - not needed if POLLSE1 is used
		wypin	#1, rp_clk-0    // ensure card releases the CMD bus
		setse1	rm_se1-0    // must be immediately after WYPIN to catch IN rising

		or	cmd, #0x40    // add S (low) and T (high) bits in front of command
		shl	cmd, #24    // command at [31:24] is needed for later CRC
		mov	pb, arg
		shr	pb, #8
		or	pb, cmd
		rev	pb    // first 32 bits prep'd for streamer
		setxfrq	##0x8000_0000UL    // sysclock/1 for lead-in timing
		waitse1        // wait for clock pulse to complete

		fltl	rp_clk-0    // reset clock-gen smartpin, needed to instruction align the samrtpin cycle
		testp	rp_cmd-0   wc    // send new command only if CMD pin has returned high
// C set when CMD pin high
	if_c	drvl	rp_cmd-0    // drive CMD pin low, start-bit, ready for streamer output

// begin command phase
	if_c	xinit	rm_align-0, #0    // lead-in delay at sysclock/1
	if_c	setq	rv_nco-0    // data transfer rate, takes effect on XZERO below (buffered command)
	if_c	xzero	rm_ca-0, pb    // place first 32 bits in tx buffer, aligned to clock via lead-in delay
		dirh	rp_clk-0    // clock timing starts here, first clock pulse occurs in smartpin's second period
		wypin	#6*8+1, rp_clk-0    // SD clock pulses, 6 bytes + 1 pulse to kick off a response

// CRC7 algorithm, SD spec 4.5
	if_c	mov	pb, #0
	if_c	setq	cmd    // first byte, CMD<<24
	if_c	crcnib	pb, #0x48    // CRC-7-ITU reversed (x7 + x3 + x0: 0x09, odd parity)
	if_c	crcnib	pb, #0x48
	if_c	setq	arg     // + four bytes
	if_c	crcnib	pb, #0x48
	if_c	crcnib	pb, #0x48
	if_c	crcnib	pb, #0x48
	if_c	crcnib	pb, #0x48
	if_c	crcnib	pb, #0x48
	if_c	crcnib	pb, #0x48
	if_c	crcnib	pb, #0x48
	if_c	crcnib	pb, #0x48

	if_c	or	pb, #0x180    // add packet end-bit after the CRC
	if_c	rev	arg    // msbit in bit0 position to suit both reverse CRC and lsbit first streamer mode
	if_c	rolbyte	pb, arg, #3    // insert last byte of arg ahead of CRC

	if_c	xzero	rm_ca-0, pb     // place remaining 16 bits in buffer for when first 32 bits completes
	if_nc	mov	cmd, #0    // CMD pin stuck low - Probably needs power down
    }
#ifdef SD_DEBUG
    if( !cmd )
        __builtin_printf(" CMD pin stuck low! ");
#endif

    return cmd;    // 0:CMD pin stuck low
}



//-----------------------------------------------------------------------
// Send a common housekeeping command packet to the SD card
//-----------------------------------------------------------------------

static int  send_cmd(    // 0:Fail, 1:Success
    uint32_t cmd,    // Command
    uint32_t arg,    // Argument
    uint8_t *resp )    // Buffer address for response bytes
{
#ifdef SD_DEBUG
//    __builtin_printf(" CMD%d - ", cmd);
#endif
    tx_command(cmd, arg);
    if( !resp ) {    // expecting no response, eg: CMD7 deselect
        uint32_t  PIN_CLK = clkpin;
        uint32_t  PIN_CMD = cmdpin;
        __asm {
		waitse1    // wait for end of command tx clocks
		outh	PIN_CMD    // momentarily force CMD pin high before new clocks
		wypin	#8, PIN_CLK    // trailing clocks
		fltl	PIN_CMD    // pin drive release before SD card takes over
		waitse1       // wait for end of command tx clocks
		wypin	#500, PIN_CLK    // trailing clocks
        }
        return 1;
    }
    if( rx_response(resp, 6, -500) )    // R1,R1b,R3,R6,R7 responses
        if( resp[5] == crc7sd(resp, 5) )
            return 1;
#ifdef SD_DEBUG
    __builtin_printf(" CMD%d error! ", cmd);
#endif
    return 0;
}



//-----------------------------------------------------------------------
// Send a R2 housekeeping command packet to the SD card
//-----------------------------------------------------------------------

static int  send_cmd_r2(    // 0:Fail, 1:Success
    uint32_t cmd,    // Command
    uint32_t arg,    // Argument
    uint8_t *resp )    // Buffer address for response bytes
{
#ifdef SD_DEBUG
//    __builtin_printf(" CMD%d - ", cmd);
#endif
    tx_command(cmd, arg);
    if( rx_response(resp, 17, -500) )    // R2 response, response length, trailing clocks
        if( resp[16] == crc7sd(&resp[1], 15) )
            return 1;
#ifdef SD_DEBUG
    __builtin_printf(" CMD%d error! ", cmd);
#endif
    return 0;
}



//-----------------------------------------------------------------------
// Send an application specific command packet to the SD card
//-----------------------------------------------------------------------

static int  send_acmd(    // 0:Fail, 1:Success
    uint32_t acmd,    // App command
    uint32_t arg,    // Argument
    uint8_t *resp )    // Buffer address for response bytes
{
    if( send_cmd(55, rca16<<16, resp) ) {    // ACMD prefix
#ifdef SD_DEBUG
//    __builtin_printf(" ACMD%d - ", acmd);
#endif
        tx_command(acmd, arg);
        if( rx_response(resp, 6, -500) )
            return 1;    // skip CRC checking since not all ACMDs have one
    }
#ifdef SD_DEBUG
    __builtin_printf(" ACMD%d error! ", acmd);
#endif
    return 0;
}



//-----------------------------------------------------------------------
// Setup the SD clock pin. Also configure DMA parameter sets
//-----------------------------------------------------------------------

static void  sd_clockconf(
    uint32_t clkdiv,    // minimum value of 2
    uint32_t rxlag )    // range 1..24
{
    uint32_t  CLK_POL = ledpin & CLK_POLARITY_MASK;

    // update the parameter sets
    set_cmdset(clkdiv, rxlag, CLK_POL);
    set_respset(clkdiv, rxlag);
    set_rxblkset(clkdiv, rxlag);
    set_txblkset(clkdiv, rxlag, CLK_POL);

    // configure pins
    uint32_t  pinmode = (rxlag&1 ? P_SYNC_IO : 0) | (RX_SCHMITT ? P_SCHMITT_A : 0);
    _wrpin(cmdpin, pinmode);
    _wrpin(dat0pin | 3<<6, pinmode);

    // SD clock-gen smartpin
    pinmode = P_PULSE | P_OE | (CLK_POL ? P_INVERT_OUTPUT : 0) | P_SCHMITT_A | (CLK_REG ? P_SYNC_IO : 0);
    _pinstart(clkpin, pinmode, clkdiv | clkdiv/2<<16, 0);
}



//-----------------------------------------------------------------------
// Rx clock-data phase alignment routine
// operates with CMD pin only but applies to DAT pins equally, for which
// I'd like remedied but don't know of one
//-----------------------------------------------------------------------
enum { PER_DOT_TRIES = 12, CID_SIZE = 17 };


static int  calibrate_rxlag(void)    // 0:Failed calibration,  1..30:New rxlag set
{
    uint8_t  *resp = __builtin_alloca(PER_DOT_TRIES * CID_SIZE);    // response buffer, in hubRAM
    uint8_t  *comp = __builtin_alloca(PER_DOT_TRIES * CID_SIZE);    // compare buffer, in hubRAM
    uint32_t  clkdiv = clkdivider;
    uint32_t  idx, pass = 0, rca = rca16 << 16;    // stored copy of the card's RCA register
    int32_t  rxlag, repeats, highest = 0, lowest = 0;

    for( idx = 0; idx < PER_DOT_TRIES * CID_SIZE; idx += CID_SIZE )
        memcpy(comp + idx, cidbytes, CID_SIZE);

    send_cmd(7, 0, NULL);    // CMD7  DESELECT_CARD - "tran" to "standby" state, no response

    for( rxlag = 1; rxlag <= 24; rxlag++ )
    {
        sd_clockconf(clkdiv, rxlag);    // adjust to next phase timings

        repeats = 80;
        do {
            memset(resp, 0, PER_DOT_TRIES * CID_SIZE);    // erase buffer before use
#ifdef SD_DEBUG
            putchar('.');
#endif
            idx = PER_DOT_TRIES * CID_SIZE;
            do {
                idx -= CID_SIZE;
                if( !send_cmd_r2(10, rca, resp + idx) )    // CMD10  SEND_CID - R2 response
                    break;
            } while( idx );

            if( memcmp(comp, resp, PER_DOT_TRIES * CID_SIZE) )    // compare stored copy of CID register
                break;

        } while( --repeats );
#ifdef SD_DEBUG
        puts("");
#endif
        if( !repeats ) {    // passed
            pass |= 1UL << rxlag;    // log a bitmap of passes
            highest = rxlag;
            if( !lowest )
                lowest = rxlag;
        }
    }

    if( highest )  {    // we have a result
        rxlag = highest - (highest - lowest) / 2;    // mid-point
        if( !((pass >> rxlag) & 1) )    // if mid-point was a fail
            rxlag--;    // adjust by one
    } else
        rxlag = 0;    // no result

    rxlagcomp = rxlag;    // update to newly selected value
    sd_clockconf(clkdiv, rxlag);    // apply selected value
#ifdef SD_DEBUG
    __builtin_printf( "  rxlag=%d selected  Lowest=%d Highest=%d\n", rxlag, lowest, highest );
#endif
    send_cmd(7, rca, resp);    // CMD7  SELECT_CARD - "standby" to "tran" state, R1b response

    return rxlag;
}



//-----------------------------------------------------------------------
// Action a new clock divider setting
//-----------------------------------------------------------------------

static int  new_clkdiv(
    uint32_t clkdiv )    // minimum value of 2
{
    if( clkdiv < 2 )  clkdiv = 2;    // bounds check
    if( clkdiv > 0xffff )  clkdiv = 0xffff;    // bounds check
    clkdivider = clkdiv;
#ifdef SD_DEBUG
    uint32_t  tmr = _clockfreq() / (clkdiv * 100_000UL);
    __builtin_printf(" SD clock-divider set to sysclock/%d (%d.%d MHz)\n",
                     clkdiv, tmr/10, tmr%10);
#endif

    return calibrate_rxlag();    // perform rx calibration, also selects card and sets full speed clock
}



//-----------------------------------------------------------------------
// Command-Response error handler
//-----------------------------------------------------------------------

static int  cr_error_handler( void )
{

    return 1;
}



//-----------------------------------------------------------------------
// Initialize SD Card
//-----------------------------------------------------------------------
enum { CMD8CHECKPAT = 0x100|0x5a };    // 3V3 bit and check pattern, SD spec 4.3.13


static int  sd_initialise( void )
{
    uint8_t  *resp = __builtin_alloca(20);    // response buffer, in hubRAM
    uint32_t  rca, tmr, a41arg, PIN_CLK = clkpin;
    int  status = STA_NOINIT | STA_NODISK;

//_pinl(56);    // diag
    if( !sdcard_power() )    // attempt to power cycle the SD card
        return status;

// setup SD clock smartpin, and other pins, and fill out initial setsets
    tmr = (_clockfreq() + 200_000UL) / 400_000UL;    // divider for 400 kHz SD clock
    rxlagcomp = 0;
    sd_clockconf(tmr, 0);
#ifdef SD_DEBUG
    __builtin_printf(" SD clock-divider set to sysclock/%d (400 kHz)\n", tmr);
#endif

// give it a kick
    _wypin(PIN_CLK, 500);    // SD spec 6.4.1
    _waitms(1);
    send_cmd(0, 0, NULL);    // CMD0  GO_IDLE_STATE - in case the power cycle didn't happen
    _waitms(1);
    send_cmd(8, CMD8CHECKPAT, resp);    // CMD8  SEND_IF_COND - 3.3V and check pattern, SD spec 4.3.13, R7 response
    _waitms(1);

// start conversing with SD card
    send_cmd(0, 0, NULL);    // CMD0  GO_IDLE_STATE - no response expected
    _waitms(1);
//    if( !send_cmd(8, CMD8CHECKPAT, resp) )    // CMD8  SEND_IF_COND - 3.3V and check pattern, SD spec 4.3.13, R7 response
//        goto faillabel;    // no response, probably card missing
    if( send_cmd(8, CMD8CHECKPAT, resp) ) {    // CMD8  SEND_IF_COND - 3.3V and check pattern, SD spec 4.3.13, R7 response
        if( __builtin_bswap32(*(uint32_t *)&resp[1]) != CMD8CHECKPAT )// confirms SD v2.0+, with valid voltage and check pattern
            goto faillabel;
        a41arg = 1<<30 | 1<<20 | 1<<28;    // set host as HCS capable, 3.3 Volt, and request XPC (full power)
    } else
        a41arg = 1<<20;    // card is not HCS capable, SD v1.0, 3.3 Volt

// still can't be sure there is actually a card pressent
    if( send_acmd(41, a41arg, resp) ) {   // ACMD41  SD_SEND_OP_COND - any response is card really exists
#ifdef SD_DEBUG
        __builtin_printf(" Card idle OK\n");
#endif
        status = STA_NOINIT;
    } else {
#ifdef SD_DEBUG
        __builtin_printf(" Card no response - ");
#endif
        goto faillabel;
    }

// find card type
    rca = 0;
    rca16 = 0;    // card "idle" state expects RCA=0 in ACMDs
    tmr = _getms();
    do {
        _waitms(1);    // card busy, polling faster than 50 ms, SD spec 4.4
        if( !send_acmd(41, a41arg, resp) )   // ACMD41  SD_SEND_OP_COND - "idle" to "ready" state, R3 response
            goto faillabel;
        rca = __builtin_bswap32(*(uint32_t *)&resp[1]);
//        if( resp[1]>>7 )    // valid Ready bit, card has switched from "idle" to "ready" state
        if( rca>>31 )    // valid Ready bit, card has switched from "idle" to "ready" state
            break;
    } while( _getms() - tmr < 1000 );    // SD spec 4.2.3, 1.0 sec timeout, probably gone "inactive"
#ifdef SD_DEBUG
    __builtin_printf(" OCR register %08x  - ",rca);
#endif
    rca >>= 30;
    if( rca != 3 )    // check CCS bit
        ledpin |= BYTE_ADDRESSING_MASK;    // byte addressed card type
#ifdef SD_DEBUG
    if( rca == 3 )
        __builtin_printf("SDHC/SDXC Card\n");    // block addressed card type
    else
        __builtin_printf("SDSC Card\n");    // byte addressed card type
#endif

//-----------------------------------------
// CMD11 voltage switch not used
//   Default 3.3V is ideal for Prop2 host
//   UHS features consequently ignored
//-----------------------------------------

    // CMD2 then CMD3 are required back-to-back.  If the sequence is
    //   unsuccesful then CMD3 will timeout and return a zero value
    send_cmd_r2(2, 0, resp);    // CMD2  ALL_SEND_CID - "ready" to "ident" state, R2 response (CID)
    if( !send_cmd(3, 0, resp) )    // CMD3  publish new RCA - "ident" to "standby" state, R6 response
        goto faillabel;

    rca = __builtin_bswap16(*(uint16_t *)&resp[1]);
    rca16 = rca;
    rca = rca << 16;
#ifdef SD_DEBUG
    __builtin_printf(" Data Transfer Mode entered - Published RCA %08x\n", rca);
#endif

    send_cmd_r2(10, rca, resp);    // CMD10  SEND_CID, R2 response
    memcpy(cidbytes, resp, 17);
#ifdef SD_DEBUG
    __builtin_printf(" CID register backed up\n");
#endif

    if( !send_cmd(7, rca, resp) )    // CMD7  SELECT_CARD - "standby" to "tran" state, R1b response
        goto faillabel;

    status = 0;   // card is good to go

    // Need 4-bit bus width for data blocks
    if( !send_acmd(6, 2, resp) )  // ACMD6  SET_BUS_WIDTH - Engage 4-bit parallel data, R1 response
        goto faillabel;
#ifdef SD_DEBUG
    __builtin_printf(" 4-bit data interface engaged\n");
#endif

    uint8_t  *buff = __builtin_alloca(512);    // data buffer, in hubRAM
    uint32_t  timeout = _clockfreq() / 4;    // 250 ms

#if FF_USE_TRIM
    if( send_acmd(13, 0, resp) ) {    // ACMD13  SEND_SD_STATUS (SSR), spec 4.10.2
        rx_datablocks(buff, 1, timeout, resp);    // data length is 64 bytes, CRC will fail

        if( (buff[24]>>1) & 1 )
            ledpin |= TRIM_DISCARD_MASK;    // TRIM support enable, via block erase "discard"ing

#ifdef SD_DEBUG
   //     __builtin_printf("ACMD13: ");
  //      for( tmr = 0; tmr <= 63; tmr++ )
 //           __builtin_printf(" %02x", buff[tmr]);
//        __builtin_printf("\n");

        tmr = buff[8] * 2;
        if( tmr > 8 )
            tmr = -1;
        if( tmr == 8 )
            tmr = 10;
        __builtin_printf(" Speed Class = C%d", tmr);    // spec 4.10.2.2
        __builtin_printf("  UHS Grade = U%d", buff[14] >> 4);    // spec 4.10.2.8
        __builtin_printf("  Video Class = V%d", buff[15]);    // spec 4.10.2.10
        __builtin_printf("  App Class = A%d\n", buff[21] & 0xf);    // spec 4.10.2.13
        __builtin_printf("  TRIM = %b  FULE = %b\n", (buff[24]>>1) & 1, buff[24] & 1);    // table 4-44, bits 313,312
#endif
    }
#endif

    a41arg = 0;
    // CMD6  SWITCH_FUNC (Spec 4.3.10 and 4.10.2):  Select High-Speed Access Mode
    //   Supposedly, this doubles the card's allotted power limit to 200 mA
    //   It notably phase shifts the clock-data relationship
    if( send_cmd(6, 0x80_fffff1, resp) ) {    // CMD6  SWITCH_FUNC, Set G1=1 (High Speed)
        // Oddly, CMD6 reads the function table as a data block
        rx_datablocks(buff, 1, timeout, resp);    // func table data length is 64 bytes, CRC will fail
//        _wypin(PIN_CLK, 20);    // SD spec 4.3.10.1, fig 4-14
        a41arg = 1;    // success flag
    }
    // Now to verify the switch function
    send_cmd(7, 0, NULL);    // CMD7  DESELECT_CARD - "tran" to "standby" state, no response
    if( !send_cmd_r2(9, rca, resp) )    // CMD9  SEND_CSD - R2 response
        goto faillabel;
    discblocks = disc_size(resp + 1);    // user area block count, C_SIZE - SD spec 5.3.3 table 5-16
#ifdef SD_DEBUG
    __builtin_printf(" Card User Capacity = %d MiB\n", discblocks / 2048);
#endif
    if( a41arg && (resp[4] == 0b0_1011_010) ) {    // 50 Mb/s: TRAN_SPEED - SD spec, table 5-6
#ifdef SD_DEBUG
        __builtin_printf(" High-Speed access mode engaged\n");
#endif
    } else {    // 32h=25MHz, 5Ah=50MHz, 0Bh=100MHz, 2Bh=200MHz
        ledpin |= CLK_POLARITY_MASK;    // set inverted clock polarity to suit Standard Speed access mode
#ifdef SD_DEBUG
        __builtin_printf(" Default-Speed access mode\n");
#endif
    }

    ledpin |= BLOCKREAD_CRC_MASK;    // block read CRC enable flag

    // Time for full speed clocking - After optional High-Speed access mode has been set.
    // The calibration process phase aligns the rx clock-data relationship - Which differs between
    // access modes.
    // Ie: It's important for timing to stay loose, (rxlagcomp = 0) while at 400 KHz, because
    // calibrating prior to setting the access mode would just mess up when the access mode gets set.
    if( !new_clkdiv(CLK_DIV_DEFAULT) )    // sets and runs the calibration for this divider value
        goto faillabel;

#ifdef SD_DEBUG
    uint8_t  *cid = &cidbytes[1];
    __builtin_printf(" CID decode:  ManID=%02x   OEMID=%c%c", cid[0], cid[1], cid[2]);
    __builtin_printf("  Name=%c%c%c%c%c\n", cid[3], cid[4], cid[5], cid[6], cid[7]);
    __builtin_printf("  Ver=%x.%x   Serial=%08x   Date=%d-%d\n", cid[8]>>4, cid[8]&0x0f,
            __builtin_bswap32(*(int32_t*)&cid[9]), 2000+((cid[13]<<4|cid[14]>>4)&0xff), cid[14]&0x0f );
/*
    //----------------------------------------
    // This is a whole SD card discard erase
    // Used only for testing
    //----------------------------------------
    if( ledpin & TRIM_DISCARD_MASK ) {
        send_cmd(32, 0, resp);    // CMD32  ERASE_WR_BLK_START
        send_cmd(33, discblocks - 1, resp);    // CMD33  ERASE_WR_BLK_END
        if( send_cmd(38, 1, resp) )    // CMD38  ERASE, 0 = Erase, 1 = Discard, 2 = FULE
            __builtin_printf("FULE command issued\n");
        else
            __builtin_printf("FULE command error\n");
        if( wait_ready(_clockfreq()*4) )    // 4 secs, check busy status on DAT0
            __builtin_printf("Card ready\n");
        else
            __builtin_printf("Card still busy\n");
    }
*/
    __builtin_printf("SD Card Init Successful\n");
#endif
    give_pins();    // pins DIRs lowered

    return status;

faillabel:
#ifdef SD_DEBUG
    __builtin_printf("SD Init Failure!  :-(\n");
#endif
    _wypin(PIN_CLK, 500);
    _waitus(20);

    // return the I/O pins to unconfigured
    releasepins();

    return STA_NOINIT;
}




//-----------------------------------------------------------------------
//
// Interface to FlexProp VFS
//
#include <errno.h>
#include <sys/types.h>
#include <sys/vfs.h>
#include <stdbool.h>

#include <filesys/fatfs/ff.h>
#include <fcntl.h>

#define BLOCK_SIZE  512
#define BLOCK_SHIFT 9
#define BLOCK_MASK  0x1ff

off_t  curpos;    // 64-bit, byte offset of current/next block number (system interface variable)
uint64_t  f_pinmask;    // 64-bit, driver's I/O pin map, for both allocating and deallocating
LBA_t  cmd12lba;    // lazy CMD12's jointing block number



static ssize_t  v_read( vfs_file_t *fil, void *buff, size_t count )
{
    uint8_t  *resp = __builtin_alloca(8);    // response buffer, in hubRAM
    LBA_t  blocks = count >> BLOCK_SHIFT;
    ssize_t  bytes_io;
    LBA_t  cmdparam, lba = curpos >> BLOCK_SHIFT;
    LBA_t  cmd12blk = cmd12lba;    // state tracking for where CMD12 to be next issued
    uint32_t  timeout = _clockfreq() / 4;    // 250 ms
    unsigned  PIN_LED = ledpin;    // Doubles up as SDSC byte addressing flag in bit0
    int  rc;

#ifdef SD_DEBUG_ACCESSES
    if( blocks == 1 )
        __builtin_printf(" cp%dRD%x ", rxblkset.p_clk, lba);
    else
        __builtin_printf(" cp%dRD%x+%x ", rxblkset.p_clk, lba, blocks);
#endif
    if( !blocks || (uint32_t)curpos & BLOCK_MASK ) {
        // we have to do the I/O for the first sector
        // for now throw up our hands and punt, we don't support arbitrary seeks
        return -1;
    }

    // disjoint
    if( (cmd12blk != lba) || !lba ) {    // detect a disjoint
        if( cmd12blk )
            send_cmd(12, 0, NULL);    // CMD12  STOP_TRANSMISSION

        cmdparam = (PIN_LED & BYTE_ADDRESSING_MASK) ? lba << BLOCK_SHIFT : lba;
        if( !wait_ready(timeout) )    // check busy-low on DAT0
            goto errorlabel;

        tx_command(18, cmdparam);    // CMD18  READ_MULTIPLE_BLOCK
        if( !rx_response(resp, 6, 2) )    // R1 response, response length, trailing clocks
            goto errorlabel;

    } else    // joint, prior CMD18 is still in operaion
        take_clkpin();    // pins DIRs get lowered at end of each driver transaction

    // joint
    PIN_LED >>= 8;    // just the pin number
    _pinl(PIN_LED);    // activity LED on
    rc = rx_datablocks(buff, blocks, timeout, resp);
    _pinf(PIN_LED);    // activity LED off

    if( rc ) {    // any error
        send_cmd(12, 0, resp);    // CMD12  STOP_TRANSMISSION
        if( rc == 1 )    // CRC mismatch
            calibrate_rxlag();    // attempt to improve situation
    }

errorlabel:
    give_pins();    // pins DIRs lowered
#ifdef SD_DEBUG_ACCESSES
    __builtin_printf("%d ", _getus());
#endif
    if( rc ) {    // error
        cmd12lba = 0;    // force a disjoint
        return 0;

    } else {
        cmd12lba = lba + blocks;    // remember next incremental block number (joint)
        bytes_io = (ssize_t)blocks << BLOCK_SHIFT;
        curpos += bytes_io;    // byte offset of same block (system interfaced variable)
        return bytes_io;
    }
}



static ssize_t  v_write( vfs_file_t *fil, void *buff, size_t count )
{
    uint8_t  *resp = __builtin_alloca(8);    // response buffer, in hubRAM
    LBA_t  blocks = count >> BLOCK_SHIFT;
    LBA_t  cmdparam, lba = curpos >> BLOCK_SHIFT;
    LBA_t  cmd12blk = cmd12lba;    // state tracking for where CMD12 to be next issued
    ssize_t  bytes_io = (ssize_t)blocks << BLOCK_SHIFT;
    uint32_t  timeout = _clockfreq() / 4;    // 250 ms
    unsigned  PIN_LED = ledpin;    // Doubles up as SDSC byte addressing flag in bit0
    int  rc;

#ifdef SD_DEBUG_ACCESSES
    if( blocks == 1 )
        __builtin_printf(" cp%dWR%x ", rxblkset.p_clk, lba);
    else
        __builtin_printf(" cp%dWR%x+%x ", rxblkset.p_clk, lba, blocks);
#endif
    if( !blocks || (uint32_t)curpos & BLOCK_MASK ) {
        // we have to do the I/O for the first sector
        // for now throw up our hands and punt, we don't support arbitrary seeks
        return -1;
    }

    cmdparam = (PIN_LED & BYTE_ADDRESSING_MASK) ? lba << BLOCK_SHIFT : lba;
    PIN_LED >>= 8;    // just the pin number

    if( blocks == 1 ) {    // some cards get pissy if using CMD25 to write one block
        lba = 0;    // without CMD25 all singles are considered as disjoints
        if( cmd12blk )
            send_cmd(12, 0, NULL);    // CMD12  STOP_TRANSMISSION

        if( wait_ready(timeout) ) {    // check busy-low on DAT0

            tx_command(24, cmdparam);    // CMD24  WRITE_SINGLE_BLOCK
            if( rx_response(resp, 6, 2) ) {    // R1 response, response length, trailing clocks

                _pinl(PIN_LED);    // activity LED on
                rc = tx_datablock(buff, timeout);
                if( rc == 0b10_010_1 )    // CRC matched
                    blocks = 0;
                _pinf(PIN_LED);    // activity LED off
            }
        }
    } else {
        // disjoint
        lba = -lba;    // to differentiate writes from reads
        if( (cmd12blk != lba) || !lba ) {    // detect a disjointed block number
            if( cmd12blk )
                send_cmd(12, 0, NULL);    // CMD12  STOP_TRANSMISSION

            if( !wait_ready(timeout) )    // check busy-low on DAT0
                goto errorlabel;

            tx_command(25, cmdparam);    // CMD25  WRITE_MULTIPLE_BLOCK
            if( !rx_response(resp, 6, 2) )    // R1 response, response length, trailing clocks
                goto errorlabel;

        } else    // joint, prior CMD25 is still in operaion
            take_clkpin();    // pins DIRs get lowered at end of each driver transaction

        // joint
        lba -= blocks;    // to update "cmd12lba", if no error
//        timeout |= 1;    // diag control to force a sent CRC to fail during CMD25

        _pinl(PIN_LED);    // activity LED on
        do {
            rc = tx_datablock(buff, timeout);
            if( rc != 0b10_010_1 )
                break;   // other than CRC matched
            buff += BLOCK_SIZE;
        } while( --blocks );
        _pinf(PIN_LED);    // activity LED off

        if( blocks )    // error
            send_cmd(12, 0, resp);    // CMD12  STOP_TRANSMISSION, also wait for the command response
    }

errorlabel:
    give_pins();    // pins DIRs lowered
#ifdef SD_DEBUG_ACCESSES
    __builtin_printf("%d ", _getus());
#endif
    if( blocks ) {    // error
        cmd12lba = 0;    // force a disjoint
        return 0;

    } else {
        cmd12lba = lba;    // remember next incremental block number (joint)
        curpos += bytes_io;    // byte offset of same block (system interfaced variable)
        return bytes_io;
    }
}



static int  v_ioctl(vfs_file_t *fil, int ctrl, void *buff)
{
    DRESULT  res = RES_OK;

    switch( ctrl )
    {
        case CTRL_SYNC :    // Make sure that no pending write process
#ifdef SD_DEBUG_ACCESSES
    __builtin_printf(" SYNC ");
#endif
            if( cmd12lba  ) {
                cmd12lba = 0;    // disjointed here
                send_cmd(12, 0, NULL);    // CMD12  STOP_TRANSMISSION
            }
            if( !wait_ready(_clockfreq()/4) )    // 250 ms, check busy status on DAT0
                res = RES_ERROR;
#ifdef SD_DEBUG_ACCESSES
    __builtin_printf("%d ", _getus());
#endif
            break;


        case GET_SECTOR_COUNT :    // Get number of (512 byte) blocks on the disc
#ifdef SD_DEBUG
    __builtin_printf(" CAPACITY ");
#endif
            *(LBA_t *)buff = discblocks;    // SD user area block count, from CSD register
            break;

#if FF_MAX_SS != FF_MIN_SS
        case GET_SECTOR_SIZE :    // Get block size
            *(LBA_t *)buff = BLOCK_SIZE;    // 512 bytes
            break;
#endif
#if FF_USE_MKFS
        case GET_BLOCK_SIZE :    // Get erase Allocation Unit (AU) size
            *(LBA_t *)buff = 128;    // 128 blocks, 64 kB, copied from sdmm.cc
            break;
#endif
#if FF_USE_TRIM
        case CTRL_TRIM :
            if( ledpin & TRIM_DISCARD_MASK ) {    // action only if discard feature is supported
                uint8_t  *resp = __builtin_alloca(8);    // response buffer, in hubRAM

                if( cmd12lba  ) {
                    cmd12lba = 0;    // disjointed here
                    send_cmd(12, 0, NULL);    // CMD12  STOP_TRANSMISSION
                }
#if defined SD_DEBUG_ACCESSES || defined SD_DEBUG
    __builtin_printf(" TRIM %x..%x ", ((LBA_t *)buff)[0], ((LBA_t *)buff)[1]);
#endif
                if( wait_ready(_clockfreq()/4) ) {    // 250 ms, check busy status on DAT0
                    send_cmd(32, ((LBA_t *)buff)[0], resp);    // CMD32  ERASE_WR_BLK_START
                    send_cmd(33, ((LBA_t *)buff)[1], resp);    // CMD33  ERASE_WR_BLK_END
                    if( !send_cmd(38, 1, resp) )    // CMD38  ERASE, 0 = Erase, 1 = Discard, 2 = FULE
                        res = RES_ERROR;
                }
#ifdef SD_DEBUG_ACCESSES
    __builtin_printf(" %d %d ", res, _getus());
#endif
            }
            break;
#endif

        case CTRL_BLOCKREAD_CRC :    // Disables/enables CRC processing of block read, default is enabled
            if( *(unsigned *)buff )
                ledpin |= BLOCKREAD_CRC_MASK;
            else
                ledpin &= ~BLOCKREAD_CRC_MASK;
#ifdef SD_DEBUG
    __builtin_printf(" BLOCK_READ_CRC %x %d ", res, ledpin&0xff);
#endif
            set_rxblkcrc();    // apply the selected exec path of rx_datablocks()
            break;


        case CTRL_GET_CLKDIV :    // Get the active clock-divider
#ifdef SD_DEBUG
    __builtin_printf(" GETDIV ");
#endif
            *(int *)buff = clkdivider;
            break;


        case CTRL_SET_CLKDIV :    // Set the active clock-divider
#ifdef SD_DEBUG
    __builtin_printf(" SETDIV ");
#endif
            new_clkdiv(*(int *)buff);
            break;


        default:
            res = RES_PARERR;
    }

    give_pins();    // pins DIRs lowered

    if( res )
        return _seterror(EINVAL);
    return 0;
}



static off_t  v_lseek( vfs_file_t *fil, off_t off, int whence )
{
    if (whence == 0) {
        curpos = off;
    } else if (whence == 1) {
        curpos += off;
    } else {
        curpos = -off;
    }
    return curpos;
}



static int  v_flush( vfs_file_t *fil )    // flush v_putc/v_getc buffers
{
#ifdef SD_DEBUG
    __builtin_printf(" FLUSH ");
#endif
    return 0;
}

static int  v_putc(int c, vfs_file_t *fil) {
    if (v_write(fil, &c, 1) == 1) return c;
    return -1;
}
static int  v_getc(vfs_file_t *fil) {
    int c = 0;
    if (v_read(fil, &c, 1) == 1) return c;
    return -1;
}



static int  v_close(vfs_file_t *fil)
{
#ifdef _DEBUG
    __builtin_printf( " Clear pins: %d %d %d %d %d\n", clkpin, cmdpin, dat0pin, pwrpin, ledpin >> 8 );
#endif
    // return the I/O pins to unconfigured
    releasepins();
    _waitms(1);

    // deallocate and exit
    _freepins(f_pinmask);
    return 0;
}



vfs_file_t *
_sdsd_open(int pclk, int pcmd, int pdat0, int ppwr, int pled)
{
    cmd12lba = 0;    // clear the jointing block number
#ifdef SD_DEBUG
    puts(" SD card driver (4-bit SD mode) "_SDSD_VERSION_);
#endif
#ifdef _DEBUG
    __builtin_printf(" sdsd_open: using pins: %d %d %d %d %d\n", pclk, pcmd, pdat0, ppwr, pled);
#endif
    uint64_t  pmask = (1ULL << pclk) | (1ULL << pcmd) | (7ULL << pdat0);
    if( ppwr >= 0 )    // optional SD slot power switch
        pmask |= 1ULL << ppwr;
    if( pled >= 0 )    // optional SD slot activity LED
        pmask |= 1ULL << pled;
    if( !_usepins(pmask) )  {
        _seterror(EBUSY);
        return NULL;
    }
    f_pinmask = pmask;    // store the alloted pin mask

    // Fill out I/O pin assignments for the physical SD card
    clkpin = pclk;
    cmdpin = pcmd;
    dat0pin = pdat0;
    if( ppwr < 0 )    // power switch pin (-1 == not defined)
        ppwr = pclk;    // use clock smartpin as dummy
    if( pled < 0 )    // activity LED pin (-1 == not defined)
        pled = pclk;    // use clock smartpin as dummy
    pwrpin = ppwr;
    ledpin = (unsigned)pled << 8;    // doubles up as SDSC byte addressing flag in bit0

    // Bring up the card, ready for mounting
    int  rc = sd_initialise();
    if( rc == 0 )  {
        vfs_file_t  *handle = _get_vfs_file_handle();
        if( handle )  {
            // We're all good, assign the FS hooks and exit
            handle->flags = O_RDWR;
            handle->bufmode = _IONBF;
            handle->state = _VFS_STATE_INUSE | _VFS_STATE_WROK | _VFS_STATE_RDOK;
            handle->read = &v_read;
            handle->write = &v_write;
            handle->close = &v_close;
            handle->ioctl = &v_ioctl;
            handle->flush = &v_flush;
            handle->lseek = &v_lseek;
            handle->putcf = &v_putc;
            handle->getcf = &v_getc;
            return handle;
        }
    }
#ifdef _DEBUG
    __builtin_printf(" sd card initialise: result=[0b%b]\n", rc);
    _waitms(1000);
#endif
    _freepins(pmask);
    _seterror(EIO);
    return NULL;
}
