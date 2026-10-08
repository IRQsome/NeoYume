/*------------------------------------------------------------------------/
/  Foolproof MMCv3/SDv1/SDv2 (in SPI mode) control module
/-------------------------------------------------------------------------/
/
/  Copyright (C) 2019, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------/
  Features and Limitations:

  * Easy to Port Bit-banging SPI
    It uses only four GPIO pins. No complex peripheral needs to be used.

  * Platform Independent
    You need to modify only a few macros to control the GPIO port.

  * Low Speed
    The data transfer rate will be several times slower than optimised for a specific processor.

  * No Media Change Detection
    Application program needs to perform a f_mount() after media change.

/-------------------------------------------------------------------------*/

// Modified for Prop2 by evanh

#include <filesys/fatfs/ff.h>
#include <filesys/fatfs/diskio.h>
#include <stdlib.h>
#include <errno.h>

/*-------------------------------------------------------------------------*/
/* Platform dependent macros and functions needed to be modified           */
/*-------------------------------------------------------------------------*/

#include <propeller.h>			/* Include device specific declareation file here */

#ifdef PIN_CLK
#error PIN_CLK definition no longer supported, use _vfs_open_sdcardx instead
#endif
#ifdef PIN_SS
#error PIN_SS definition no longer supported, use _vfs_open_sdcardx instead
#endif

int _pin_clk;
int _pin_ss;
int _pin_di;
int _pin_do;
int _waitx_delay;

#ifdef __propeller2__
#define _prop2_mode_eh /* enable Evan Hillas's optimised code */
#endif

#ifdef _prop2_mode_eh

#define	CS_H()		_pinh(PIN_SS)	// Set MMC CS "high"
#define CS_L()		_pinl(PIN_SS)	// Set MMC CS "low"

#else
// 
// The values here were tested empirically to work at 300 MHz with some cards
// but it would be nice to have some "reason" for them :)
//
#define PAUSE()      (_waitx(16))
#define SHORTPAUSE() (_waitx(8))

#define DO_INIT()	_fltl(PIN_DO)				/* Initialize port for MMC DO as input */
#define DO		(SHORTPAUSE(), (_pinr(PIN_DO) & 1))	/* Test for MMC DO ('H':true, 'L':false) */

#define DI_INIT()	_dirh(PIN_DI)	/* Initialize port for MMC DI as output */
#define DI_H()		(_pinh(PIN_DI))	/* Set MMC DI "high" */
#define DI_L()		(_pinl(PIN_DI))	/* Set MMC DI "low" */

#define CK_INIT()	_dirh(PIN_CLK)	/* Initialize port for MMC SCLK as output */
#define CK_H()		(_pinh(PIN_CLK), PAUSE())	/* Set MMC SCLK "high" */
#define	CK_L()		(_pinl(PIN_CLK), PAUSE())	/* Set MMC SCLK "low" */

#define CS_INIT()	_dirh(PIN_SS)	/* Initialize port for MMC CS as output */
#define	CS_H()		(_pinh(PIN_SS), PAUSE())	/* Set MMC CS "high" */
#define CS_L()		(_pinl(PIN_SS), PAUSE())	/* Set MMC CS "low" */
#endif


static
void dly_us (UINT n)	/* Delay n microseconds (avr-gcc -Os) */
{
    _waitus( n );
}



/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* MMC/SD command (SPI mode) */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define CMD13	(13)		/* SEND_STATUS */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

DSTATUS Stat /*= STA_NOINIT*/;	/* Disk status */

BYTE CardType;			/* b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing */



/*-----------------------------------------------------------------------*/
/* Transmit bytes to the card (bitbanging)                               */
/*-----------------------------------------------------------------------*/

static
void xmit_mmc (
    const BYTE *buff,    // Data to be sent
    UINT bc    // Number of bytes to send
)
{
    int  PIN_CLK = _pin_clk;
    int  PIN_DI = _pin_di;
    int  d;

#ifdef _prop2_mode_eh
    int  delay = _waitx_delay;

    __asm volatile {    // "const" enforces XIP, "volatile" enforces Fcache
		rdfast	#0, buff
.tx_loop
		rfbyte	d
		rol	d, #25   wc

		rep	@.rend, #8
		drvc	PIN_DI
		drvl	PIN_CLK
		waitx	delay
		rol	d, #1   wc
		drvh	PIN_CLK
		waitx	delay
.rend
		djnz	bc, #.tx_loop

		drvh	PIN_DI
    }
#else        
	do {
		d = *buff++;	/* Get a byte to be sent */
		if (d & 0x80) DI_H(); else DI_L();	/* bit7 */
		CK_H(); CK_L();
		if (d & 0x40) DI_H(); else DI_L();	/* bit6 */
		CK_H(); CK_L();
		if (d & 0x20) DI_H(); else DI_L();	/* bit5 */
		CK_H(); CK_L();
		if (d & 0x10) DI_H(); else DI_L();	/* bit4 */
		CK_H(); CK_L();
		if (d & 0x08) DI_H(); else DI_L();	/* bit3 */
		CK_H(); CK_L();
		if (d & 0x04) DI_H(); else DI_L();	/* bit2 */
		CK_H(); CK_L();
		if (d & 0x02) DI_H(); else DI_L();	/* bit1 */
		CK_H(); CK_L();
		if (d & 0x01) DI_H(); else DI_L();	/* bit0 */
		CK_H(); CK_L();
	} while (--bc);
#endif        
}



/*-----------------------------------------------------------------------*/
/* Receive bytes from the card (bitbanging)                              */
/*-----------------------------------------------------------------------*/

static
void rcvr_mmc (
    BYTE *buff,    // Pointer to read buffer
    UINT bc    // Number of bytes to receive
)
{
    int  PIN_CLK = _pin_clk;
    int  PIN_DO = _pin_do;
    int  r;

#ifdef _prop2_mode_eh

    int  delay = _waitx_delay;

    __asm volatile {    // "const" enforces XIP, "volatile" enforces Fcache
		wrfast	#0, buff
.rx_loop
		drvl	PIN_CLK
		nop
		waitx	delay
		drvh	PIN_CLK

		rep	@.rend, #7
		waitx	delay
		drvl	PIN_CLK
		testp	PIN_DO   wc
		waitx	delay
		drvh	PIN_CLK
		rcl	r, #1
.rend
		waitx	delay
		nop
		testp	PIN_DO   wc
		rcl	r, #1
		wfbyte	r
		djnz	bc, #.rx_loop
    }
#else
	DI_H();	/* Send 0xFF */

	do {
		r = 0;	 if (DO) r++;	/* bit7 */
		CK_H(); CK_L();
		r <<= 1; if (DO) r++;	/* bit6 */
		CK_H(); CK_L();
		r <<= 1; if (DO) r++;	/* bit5 */
		CK_H(); CK_L();
		r <<= 1; if (DO) r++;	/* bit4 */
		CK_H(); CK_L();
		r <<= 1; if (DO) r++;	/* bit3 */
		CK_H(); CK_L();
		r <<= 1; if (DO) r++;	/* bit2 */
		CK_H(); CK_L();
		r <<= 1; if (DO) r++;	/* bit1 */
		CK_H(); CK_L();
		r <<= 1; if (DO) r++;	/* bit0 */
		CK_H(); CK_L();
		*buff++ = r;			/* Store a received byte */
	} while (--bc);
#endif        
}



/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
int wait_ready (void)	/* 1:OK, 0:Timeout */
{
	BYTE *d = __builtin_alloca(1);
	UINT tmr, tmout;

	tmr = _cnt();
	tmout = _clockfreq() >> 1;  // 500 ms timeout
	for(;;) {
		rcvr_mmc( d, 1 );
		if( *d == 0xFF )  return 1;
		if( _cnt() - tmr >= tmout )  return 0;
	}
}



/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static
void deselect (void)
{
	BYTE *d = __builtin_alloca(1);
	int PIN_SS = _pin_ss;

	CS_H();				/* Set CS# high */
	rcvr_mmc(d, 1);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
}



/*-----------------------------------------------------------------------*/
/* Select the card and wait for ready                                    */
/*-----------------------------------------------------------------------*/

static
int select (void)	/* 1:OK, 0:Timeout */
{
	BYTE *d = __builtin_alloca(1);
	int PIN_SS = _pin_ss;

	CS_L();			/* Set CS# low */

	rcvr_mmc(d, 1);	/* Dummy clock (force DO enabled) */
	if (wait_ready()) return 1;	/* Wait for card ready */

	deselect();
	return 0;			/* Failed */
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from the card                                   */
/*-----------------------------------------------------------------------*/

static
int rcvr_datablock (	/* 1:OK, 0:Failed */
    BYTE *buff,			/* Data buffer to store received data */
    UINT btr			/* Longword count */
)
{
    BYTE *d = __builtin_alloca(2);
    UINT tmr, tmout;

    tmr = _cnt();
    tmout = _clockfreq() >> 3;  // 125 ms timeout
    for(;;) {
        rcvr_mmc( d, 1 );
        if( d[0] != 0xFF )  break;
        if( _cnt() - tmr >= tmout )  break;
    }
    if (d[0] != 0xFE) return 0;		/* If not valid data token, return with error */

#ifdef _prop2_mode_eh
    int  PIN_CLK = _pin_clk;
    int  PIN_DO = _pin_do;

    __asm volatile {    // "const" enforces XIP, "volatile" enforces Fcache
		wrfast	#0, buff
.rx_loop
		drvl	PIN_CLK
		nop
		drvh	PIN_CLK

		rep	@.rend, #31
		drvl	PIN_CLK
		rcl	pa, #1
		drvh	PIN_CLK
		testp	PIN_DO   wc
.rend
		rcl	pa, #1
		waitx	#2
		testp	PIN_DO   wc
		rcl	pa, #1
		movbyts	pa, #0b00_01_10_11
		wflong	pa
		djnz	btr, #.rx_loop
    }
#else
    rcvr_mmc(buff, btr<<2);			/* Receive the data block into buffer */
#endif
    rcvr_mmc(d, 2);				/* Discard CRC */

    return 1;				/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to the card                                        */
/*-----------------------------------------------------------------------*/

static
int xmit_datablock (	/* 1:OK, 0:Failed */
    const BYTE *buff,	/* 512 byte data block to be transmitted */
    BYTE token			/* Data/Stop token */
)
{
    BYTE *d = __builtin_alloca(4);


    if (!wait_ready()) return 0;

    d[0] = token;
    xmit_mmc(d, 1);				/* Xmit a token */
    if (token != 0xFD) {		/* Is it data token? */
#ifdef _prop2_mode_eh
        int  PIN_CLK = _pin_clk;
        int  PIN_DI = _pin_di;

        __asm volatile {    // "const" enforces XIP, "volatile" enforces Fcache
		rdfast	#0, buff
		mov	pb, #512/4
.tx_loop
		rflong	pa
		movbyts	pa, #0b00_01_10_11

		rep	@.rend, #32
		rol	pa, #1   wc
		drvl	PIN_CLK
		drvc	PIN_DI
		drvh	PIN_CLK
.rend
		djnz	pb, #.tx_loop

		drvh	PIN_DI
        }
#else
        xmit_mmc(buff, 512);	/* Xmit the 512 byte data block to MMC */
#endif
        rcvr_mmc(d, 3);			/* Xmit dummy CRC (0xFF,0xFF) */
        //rcvr_mmc(d, 1);			/* Receive data response */
        if ((d[2] & 0x1F) != 0x05)	/* If not accepted, return with error */
            return 0;
    }

    return 1;
}



/*-----------------------------------------------------------------------*/
/* Send a command packet to the card                                     */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (		/* Returns command response (bit7==1:Send failed)*/
	BYTE cmd,		/* Command byte */
	DWORD arg		/* Argument */
)
{
	BYTE n, *buf = __builtin_alloca(8);


	if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		cmd &= 0x7F;
		n = send_cmd(CMD55, 0);
		if (n > 1) return n;
	}

	/* Select the card and wait for ready except to stop multiple block read */
	if (cmd != CMD12) {
		deselect();
		if (!select()) return 0xFF;
	}

	/* Send a command packet */
	buf[0] = 0x40 | cmd;			/* Start + Command index */
	#ifdef __propeller2__
	*(DWORD*)(buf+1) = __builtin_bswap32(arg);
	#else
	buf[1] = (BYTE)(arg >> 24);		/* Argument[31..24] */
	buf[2] = (BYTE)(arg >> 16);		/* Argument[23..16] */
	buf[3] = (BYTE)(arg >> 8);		/* Argument[15..8] */
	buf[4] = (BYTE)arg;				/* Argument[7..0] */
	#endif
	n = 0x01;						/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;		/* (valid CRC for CMD0(0)) */
	if (cmd == CMD8) n = 0x87;		/* (valid CRC for CMD8(0x1AA)) */
	buf[5] = n;
	xmit_mmc(buf, 6);

	/* Receive command response */
	if (cmd == CMD12) rcvr_mmc(buf+6, 1);	/* Skip a stuff byte when stop reading */

	n = 10;					/* Wait for a valid response in timeout of 10 attempts */
	do
		rcvr_mmc(buf+6, 1);
	while ((buf[6] & 0x80) && --n);

	return buf[6];			/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv			/* Drive number (always 0) */
)
{
	if (drv) return STA_NOINIT;

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive nmuber (0) */
)
{
	BYTE n, ty, cmd, *buf = __builtin_alloca(12);
	DSTATUS s;
	UINT tmr, retries = 5;
	int PIN_CLK = _pin_clk;
	int PIN_SS = _pin_ss;
	int PIN_DI = _pin_di;
	int PIN_DO = _pin_do;

	Stat = STA_NOINIT;

#ifdef _DEBUG_SDMM
	__builtin_printf("disk_initialize: PINS=%d %d %d %d\n", PIN_CLK, PIN_SS, PIN_DI, PIN_DO);
#endif
	if (drv) {
#ifdef _DEBUG_SDMM
		__builtin_printf("bad drv %d\n", drv);
#endif
		return RES_NOTRDY;
	}

restart:
	dly_us(1000);			/* 1ms */

#ifdef _prop2_mode_eh
	_dirl( PIN_SS );
	_wrpin( PIN_SS, 0 );
	_pinh( PIN_SS );    // Deselect SD card
	_wrpin( PIN_DO, P_HIGH_15K | P_LOW_15K );  // config for 15 k pull-up
	_pinh( PIN_DO );    // engage pull-up
	_wrpin( PIN_DI, 0 );
	_pinh( PIN_DI );
	_wrpin( PIN_CLK, 0 );
	_pinh( PIN_CLK );
	tmr = (_clockfreq() + 400_000) / 800_000UL;    // 400 kHz, halfed
	_waitx_delay = tmr < 6 ? 0 : tmr - 6;
#ifdef _DEBUG_SDMM
	__builtin_printf("400 kHz step = %d\n", tmr);
#endif
#else
	CS_INIT(); CS_H();		/* Initialize port pin tied to CS */
	CK_INIT(); CK_L();		/* Initialize port pin tied to SCLK */
	DI_INIT();				/* Initialize port pin tied to DI */
	DO_INIT();				/* Initialize port pin tied to DO */
#endif

    dly_us(1000);    // 1ms
    for(tmr = 0; tmr <= 40; tmr++)
        rcvr_mmc(buf, 10);    // Apply 80 dummy clocks and the card gets ready to receive command

    send_cmd(CMD0, 0);    // Enter Idle state
    send_cmd(CMD8, 0x1AA);
    rcvr_mmc(buf, 4);
    deselect();
    dly_us(100);
    rcvr_mmc(buf, 10);  // Apply 80 dummy clocks and the card gets ready to receive command

    ty = 0;
    if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
        rcvr_mmc(buf, 10);  // Apply 80 dummy clocks and the card gets ready to receive command
        deselect();
        dly_us(100);
#ifdef _DEBUG_SDMM
        __builtin_printf("idle OK\n");
#endif

        if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
            rcvr_mmc(buf, 4);			/* Get trailing return value of R7 resp */
#ifdef _DEBUG_SDMM
        __builtin_printf("  CMD8  R7=$%x\n", __builtin_bswap32(*(uint32_t *)buf));
#endif
            if (buf[2] == 0x01 && buf[3] == 0xAA) {		/* The card can work at vdd range of 2.7-3.6V */
                for (tmr = 1000; tmr; tmr--) {			/* Wait for leaving idle state (ACMD41 with HCS bit) */
                    if (send_cmd(ACMD41, 1UL << 30) == 0) {
                        tmr = send_cmd(CMD58, 0);	/* Check CCS bit in the OCR */
                        rcvr_mmc(buf, 4);
#ifdef _DEBUG_SDMM
        __builtin_printf("  CMD58  R1=$%x  OCR=$%x\n", tmr, __builtin_bswap32(*(uint32_t *)buf));
#endif
                        if(buf[0] & 0x80) {
//                            ty = (buf[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
                            ty = CT_SD2 | CT_BLOCK;	/* SDv2 */
                            if(buf[0] & 0x40)
                                break;
                            if(--retries)
                                goto restart;
                            else
                                break;
                        }
                    }
                    dly_us(1000);
                }
//	putchar('9');
            }
        } else {			/* SDv1 or MMCv3 */
            if (send_cmd(ACMD41, 0) <= 1) {
                ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
            } else {
                ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
            }
            for (tmr = 1000; tmr; tmr--) {			/* Wait for leaving idle state */
                if (send_cmd(cmd, 0) == 0) break;
                dly_us(1000);
            }
            if (!tmr || send_cmd(CMD16, 512) != 0) {/* Set R/W block length to 512 */
                //printf("tmr = %d\n", tmr);
                ty = 0;
            }
        }
    }

#ifdef _DEBUG_SDMM
    tmr = _clockfreq() / (8 * 100_000);
    __builtin_printf("type = %d,  Data Clock Rate = %d.%d MHz\n", ty, tmr/10, tmr%10);
#endif
    _waitx_delay = 0;
    CardType = ty;
    s = ty ? 0 : STA_NOINIT;
    Stat = s;

    deselect();

    return s;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,			/* Physical drive nmuber (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	LBA_t sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..128) */
)
{
	BYTE cmd;
	DWORD sect = (DWORD)sector;

#ifdef _DEBUG
        __builtin_printf("disk_read: block#%d count=%d ", sect, count);
#endif

	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;
	if (!(CardType & CT_BLOCK)) sect *= 512;	/* Convert LBA to byte address if needed */

	cmd = count > 1 ? CMD18 : CMD17;			/*  READ_MULTIPLE_BLOCK : READ_SINGLE_BLOCK */
	if (send_cmd(cmd, sect) == 0) {
		do {
			if (!rcvr_datablock(buff, 512/4)) break;
			buff += 512;
		} while (--count);
		if (cmd == CMD18) send_cmd(CMD12, 0);	/* STOP_TRANSMISSION */
	}
	deselect();
#ifdef _DEBUG
        __builtin_printf(" remain=%d\n", count);
#endif

	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	LBA_t sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..128) */
)
{
	DWORD sect = (DWORD)sector;

#ifdef _DEBUG
        __builtin_printf("disk_write: block#%d count=%d ", sect, count);
#endif

	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;
	if (!(CardType & CT_BLOCK)) sect *= 512;	/* Convert LBA to byte address if needed */

	if (count == 1) {	/* Single block write */
		if ((send_cmd(CMD24, sect) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sect) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	deselect();
#ifdef _DEBUG
        __builtin_printf(" remain=%d\n", count);
#endif

	return count ? RES_ERROR : RES_OK;
}

static inline LBA_t disc_size(
    uint8_t *csd )
{
    uint32_t  cs = __builtin_bswap32(*(uint32_t*)(&csd[6]));
    unsigned n;
    if( csd[0]>>6 == 1) { // SDC ver 2.00
        cs = (cs & 0x3fffff)+1;
        n = 10;
    } else {    // SDC ver 1.00
        cs = (LBA_t)(cs>>14 & 0xfff)+1;
        n = (csd[5] & 15) + (__builtin_bswap32(*(uint16_t*)(&csd[9]))>>23 & 0x7)+(2-9);
    }
    return (LBA_t)cs << n;    // 32/64-bit block count
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	BYTE n, csd[16];
	DWORD cs;

	if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;	/* Check if card is in the socket */

	res = RES_ERROR;
	switch (ctrl) {
		case CTRL_SYNC :		/* Make sure that no pending write process */
			if (select()) res = RES_OK;
			break;

		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16/4)) {
               			*(LBA_t*)buff = disc_size(csd);
				res = RES_OK;
			}
			break;

		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
			*(DWORD*)buff = 128;
			res = RES_OK;
			break;

		default:
			res = RES_PARERR;
	}

	deselect();

	return res;
}

DRESULT disk_setpins(int drv, int pclk, int pss, int pdi, int pdo)
{
    if (drv != 0) return -1;
    _pin_clk = pclk;
    _pin_ss  = pss;
    _pin_di = pdi;
    _pin_do = pdo;
#ifdef _DEBUG_SDMM
    __builtin_printf("&_pin_clk=%x, _pin_clk = %d\n", (unsigned)&_pin_clk, _pin_clk);
#endif
    return 0;
}

//
// new routine: deinitialize (clean up pins, if necessary)
//
DSTATUS disk_deinitialize( BYTE drv )
{
    int PIN_CLK = _pin_clk;
    int PIN_SS = _pin_ss;
    int PIN_DI = _pin_di;
    int PIN_DO = _pin_do;
    if (drv) {
#ifdef _DEBUG_SDMM
        __builtin_printf("deinitialize: bad drv %d\n", drv);
#endif
        return RES_NOTRDY;
    }
#ifdef _prop2_mode_eh
#ifdef _DEBUG_SDMM
    __builtin_printf("clear pins %d %d %d %d\n", PIN_CLK, PIN_SS, PIN_DI, PIN_DO);
#endif
    _pinclear(PIN_DO);
    _pinclear(PIN_DI);
    _pinclear(PIN_CLK);
    _pinclear(PIN_SS);

    _waitms(10);
#endif
    return 0;
}

/*
 * Interface to FlexProp VFS
 */
#include <sys/types.h>
#include <sys/vfs.h>
#include <stdbool.h>

#include <fcntl.h>

#define BLOCK_SIZE  512
#define BLOCK_SHIFT 9
#define BLOCK_MASK  0x1ff

off_t curpos;
uint64_t f_pinmask;

ssize_t v_do_io(vfs_file_t *fil, void *buf_p, size_t count, bool is_write)
{
    unsigned char *buf = (unsigned char *)buf_p;
    unsigned startoff = curpos & BLOCK_MASK;
    unsigned blocks;
    ssize_t  bytes_io = 0;
    unsigned lba = curpos >> BLOCK_SHIFT;
    int res;
    
    if (startoff) {
        /* we have to do the I/O for the first sector */
        /* for now throw up our hands and punt, we don't support arbitrary seeks */
        return -1;
    }
    blocks = count >> BLOCK_SHIFT;
    if (is_write) {
        res = disk_write(0, buf, lba, blocks);
    } else {
        res = disk_read(0, buf, lba, blocks);
    }
    if (res == RES_OK) {
        unsigned update = blocks << BLOCK_SHIFT;
        bytes_io += update;
        count -= update;
        curpos += update;
    }
    if (count) {
        // handle the trailing bytes;
        // not supported for now, I/O must be on block boundaries
    }
    return bytes_io;
}

ssize_t v_read(vfs_file_t *fil, void *buf, size_t count)
{
    return v_do_io(fil, buf, count, false);
}
ssize_t v_write(vfs_file_t *fil, void *buf, size_t count)
{
    return v_do_io(fil, buf, count, true);
}

int v_ioctl(vfs_file_t *fil, int arg, void *buf)
{
    DRESULT res;
    res = disk_ioctl(0, arg, buf);
    if (res)
        return _seterror(EINVAL);
    return 0;
}

off_t v_lseek(vfs_file_t *fil, off_t off, int whence)
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

int v_flush(vfs_file_t *fil)
{
    return 0;
}

int v_close(vfs_file_t *fil)
{
    disk_deinitialize(0);
    _freepins(f_pinmask);
    return 0;
}

int v_putc(int c, vfs_file_t *fil) {
    if (v_write(fil, &c, 1) == 1) return c;
    return -1;
}
int v_getc(vfs_file_t *fil) {
    int c = 0;
    if (v_read(fil, &c, 1) == 1) return c;
    return -1;
}



vfs_file_t *
_sdmm_open(int pclk, int pss, int pdi, int pdo)
{
    int r;
    int drv = 0;
    unsigned long long pmask;
    vfs_file_t *handle;

#ifdef _DEBUG
    __builtin_printf("sdmm2_open: using pins: %d %d %d %d\n", pclk, pss, pdi, pdo);
#endif    
    pmask = (1ULL << pclk) | (1ULL << pss) | (1ULL << pdi) | (1ULL << pdo);
    if (!_usepins(pmask)) {
        _seterror(EBUSY);
        return 0;
    }
    f_pinmask = pmask;
    r = disk_setpins(drv, pclk, pss, pdi, pdo);
    if (r == 0)
        r = disk_initialize(0);
    if (r != 0) {
#ifdef _DEBUG
       __builtin_printf("sd card initialize: result=[%d]\n", r);
       _waitms(1000);
#endif
       goto cleanup_and_out;
    }
    handle = _get_vfs_file_handle();
    if (!handle) goto cleanup_and_out;

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

cleanup_and_out:
    _freepins(pmask);
    _seterror(EIO);
    return 0;
}

