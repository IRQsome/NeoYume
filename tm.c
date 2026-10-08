#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void usage()
{

    puts("tm: decoder for TMDS binary data");
    puts("\tUsage:");
    puts("\t\t tm <red_bit> <green_bit> <blue_bit> <clk_bit> [<dbg_bit>]");
    puts("\t\t   where each <xxx_bit> indicates which bit of the byte");
    puts("\t\t   contains the data for channel xxx");
    puts("\t\t   and optional <dbg_bit> can be used for");
    puts("\t\t   helping identifying events like vsync etc");
        
    exit(0);
}

int terc4_decode(int val)
{
    switch(val)
    {
        case 0x29c: return 0;
        case 0x263: return 1;
        case 0x2e4: return 2;
        case 0x2e2: return 3;
        case 0x171: return 4;
        case 0x11e: return 5;
        case 0x18e: return 6;
        case 0x13c: return 7;
        case 0x2cc: return 8;
        case 0x139: return 9;
        case 0x19c: return 10;
        case 0x2c6: return 11;
        case 0x28e: return 12;
        case 0x271: return 13;
        case 0x163: return 14;
        case 0x2c3: return 15;

    }
    return -1;
}

/*
   int    %10_1001_1100 << 22   ' TERC4 0000
   int    %10_0110_0011 << 22   ' TERC4 0001
   int    %10_1110_0100 << 22   ' TERC4 0010
   int    %10_1110_0010 << 22   ' TERC4 0011
   int    %01_0111_0001 << 22   ' TERC4 0100
   int    %01_0001_1110 << 22   ' TERC4 0101
   int    %01_1000_1110 << 22   ' TERC4 0110
   int    %01_0011_1100 << 22   ' TERC4 0111
   int    %10_1100_1100 << 22   ' TERC4 1000
   int    %01_0011_1001 << 22   ' TERC4 1001
   int    %01_1001_1100 << 22   ' TERC4 1010
   int    %10_1100_0110 << 22   ' TERC4 1011
   int    %10_1000_1110 << 22   ' TERC4 1100
   int    %10_0111_0001 << 22   ' TERC4 1101
   int    %01_0110_0011 << 22   ' TERC4 1110
   int    %10_1100_0011 << 22   ' TERC4 1111

 */


int ctrl_decode(int val)
{
    switch(val)
    {
        case 0x354: return 0;  // 1101010100
        case 0xAB : return 1;  // 0010101011
        case 0x154: return 2;  // 0101010100
        case 0x2AB: return 3;  // 1010101011
    }
    return -1;
}

int once = 0;
int printbin(int,int);
int tmdsdecode(int val)
{
    int i;
    int xnor = (val & 0x100) == 0; // true if using xnor
    int d, q;
    int debug = (val == 0x2f0 && once == 0);

    if (val & 0x200) // invert data first
        val = ~val & 0xFF;

    d = (xnor ? ~val : val) << 1;
    q = val & 1;
    if (0) //(debug)
    {
        printf("\nval=%02x   d=%02x     q=%02x\n", val, d,q);
        printbin(val,8);
        printf(" ");
        printbin(d,8);
        printf(" ");
        printbin(q,8);
        printf("\n");
    }
    for (i = 1; i < 8; i++)
    {
        q |= ((val ^ d) & (1<<i));
        if (0) //(debug)
        {
            printf("val=%02x   d=%02x     q=%02x %d\n", val, d,q,i);
            printbin(val,8);
            printf(" ");
            printbin(d,8);
            printf(" ");
            printbin(q,8);
            printf("\n");
            printf("i=%i, q=%02x\n", i,q);
            once = 1;
        }
    }

    return q;
}

int printbin(int val, int bits)
{
    int i = 0;

    while(bits--)
        i+=printf("%d", (val & (1<<bits)) ? 1: 0);

    return i;
}

int parity32(unsigned int d)
{
    int i, parity = 0;

    for (i=0; i<32;i++)
        if (d & (1<<i))
            parity++;
    return parity & 1;
}

void decode_packet(int r,int g,int b)
{
    static int newpacket = 0;
    static unsigned int subpkt[4][2] = {0};
    static unsigned int ecc[4] = {0,0,0,0};
    static unsigned int checkecc[4] = {0,0,0,0};
    static unsigned int pkthdr = 0;
    static unsigned int ecchdr = 0;
    static unsigned int checkecchdr = 0;
    static unsigned int bit = 0;
    static unsigned char chnlstatl[192]={0};
    static unsigned char chnlstatr[192]={0};
    static unsigned int chnlpos= 0;
    unsigned int left, right;
    int leftp, rightp;
    int i; 
    unsigned int val, checksum, length;
    unsigned int flags;
    {
        if ((b & 8) == 0) // new packet flag
            newpacket = 1;

        if (bit < 16)
            for (i = 0; i<4; i++)
            {
                subpkt[i][0]= subpkt[i][0] + (((r & (1<<i))?1:0) << (2*bit+1)) + (((g & (1<<i))?1:0) << (2*bit));
                checkecc[i] = (checkecc[i]>>1) ^ (((checkecc[i] & 0x1) ^ ((g >> i) & 0x1))?0x83 : 0);
                checkecc[i] = (checkecc[i]>>1) ^ (((checkecc[i] & 0x1) ^ ((r >> i) & 0x1))?0x83 : 0);
            }
        else if (bit < 28)
            for (i = 0; i<4; i++)
            {
                subpkt[i][1]= subpkt[i][1] + (((r & (1<<i))?1:0) << (2*(bit-16)+1)) + (((g & (1<<i))?1:0) << (2*(bit-16)));
                checkecc[i] = (checkecc[i]>>1) ^ (((checkecc[i] & 0x1) ^ ((g >> i) & 0x1))?0x83 : 0);
                checkecc[i] = (checkecc[i]>>1) ^ (((checkecc[i] & 0x1) ^ ((r >> i) & 0x1))?0x83 : 0);
            }
        else if (bit < 32)
        {
            for (i = 0; i<4; i++)
            {
                ecc[i] = ecc[i] + (((r & (1<<i))?1:0) << (2*(bit-28)+1)) + (((g & (1<<i))?1:0) << (2*(bit-28)));
            }
        }
        if (bit < 24)
        {
            pkthdr = pkthdr + (((b & 4)?1:0)<<bit);
            checkecchdr = (checkecchdr>>1) ^ (((checkecchdr & 0x1) ^ ((b >> 2) & 0x1))?0x83 : 0);
        }
        else
            ecchdr = ecchdr + (((b & 4)?1:0)<<(bit-24));
        bit++;

        if (bit == 32)
        {
            printf("TERC4 Packet Decoded:\n");
            printf("Pkthdr:           %06X  ECC=%02X  ComputedECC=%02X\n", pkthdr, ecchdr, checkecchdr&0xff);
            if ((checkecchdr & 0xff) != (ecchdr &0xff))
                printf("Error:Bad ECC on header!\n");
            for (i=0; i<4;i++)
            {
                printf("Subpkt%d:  %06X%08X  ECC=%02X  ComputedECC=%02X\n", i, subpkt[i][1], subpkt[i][0], ecc[i], checkecc[i]&0xff);
                if ((checkecc[i] & 0xff) != (ecc[i] &0xff))
                    printf("Error:Bad ECC on Subpacket %d!\n", i);
            }

            if ((pkthdr & 0xFF) == 1) 
            {
                printf("Clock regen packet\n");
                for (i=0; i<4; i++)
                {
                    printf("  SubPkt%d    N = %-8u ", i, (subpkt[i][1]>>16) + (subpkt[i][1]&0xFF00) + ((subpkt[i][1]&0xff)<<16));
                    printf("CTS = %-8u\n", (subpkt[i][0]>>24) + ((subpkt[i][0]>>8)&0xFF00) + ((subpkt[i][0]&0xFF00)<<8));
                }
            }
            if ((pkthdr & 0xFF) == 0x84 || (pkthdr & 0xFF) == 0x82)
            {
                length = (pkthdr >> 16) & 0xff;
                printf("%s Infoframe packet\n", ((pkthdr & 0xff) == 0x82) ? "Video":"Audio");
                printf("  Version:%d\n", (pkthdr >> 8)&0xff);
                printf("  Length :%d\n", length);
                printf("  Chksum :%02X\n", subpkt[0][0]&0xff);
                checksum = pkthdr & 0xff;
                checksum += (pkthdr >> 8)&0xff;
                checksum += (pkthdr >> 16)&0xff;
                //printf("Checksum init = %02X\n", checksum & 0xff);
                printf("  Data: (LSB First) ");
                for (i=1;i<(length+1) && (i<28); i++)
                {
                    val = (subpkt[i/7][(i%7)/4] >> ((i%4)*8)) & 0xff;
                    if (i < length+1)
                    {
                        checksum += val;
                        //printf("i=%d, add %02X now %02X | ", i, val, checksum&0xff);
                    }
                    printf("%02X ", val);
                }
                puts("");
                if (((checksum &0xff)+ ((subpkt[0][0]) & 0xff)) & 0xff)
                    printf("Error: Checksum mismatch, should be %02X\n", checksum & 0xff);
            }

            if ((pkthdr & 0xFF) == 2) 
            {
                printf("Audio packet\n");
                for (i=0; i<4 && (pkthdr & (1<<(i+8))); i++)
                {
                    left = subpkt[i][0]&0xFFFFFF;
                    right = (subpkt[i][0]>>24)+((subpkt[i][1]&0xFFFF)<<8);
                    flags= (subpkt[i][1]>>16) & 0xFF;
                    printf("  Sample %d  right = %06X, left = %06X, flags=%02X  ",i, right, left, flags);
                    leftp = parity32(left);
                    rightp = parity32(right);
                    printf("Pr=%d Cr=%d, Pl=%d Cl=%d ParityR=%d ParityL=%d", flags&0x80?1:0, flags&0x40?1:0, flags&8?1:0, flags&4?1:0, rightp, leftp);
                    printf(" Channel status count=%d\n", chnlpos);
                    if (leftp != (((flags&8)?1:0) ^ ((flags&4)?1:0)))
                        printf("Error:Bad left parity!\n");
                    if (rightp != (((flags&0x80)?1:0) ^ ((flags&0x40)?1:0)))
                        printf("Error:Bad right parity!\n");
                    if (pkthdr&(1<<(20+i)))
                        chnlpos = 0;
                    if (chnlpos < 192)
                    {
                        chnlstatr[chnlpos]=(flags&0x40)?1:0;
                        chnlstatl[chnlpos]=(flags&4)?1:0;
                        chnlpos++;
                    }
                    if (chnlpos == 192)
                    { 
                        printf("L Channel Status:\n");
                        for (i = 0; i<192; i++)
                        {
                            printf("%d",chnlstatl[191-i]);
                            if ((i+1) % 8 == 0)
                                printf("_");
                            if ((i+1) % 64 == 0)
                                puts("");
                        }
                        printf("R Channel Status:\n");
                        for (i = 0; i<192; i++)
                        {
                            printf("%d",chnlstatr[191-i]);
                            if ((i+1) % 8 == 0)
                                printf("_");
                            if ((i+1) % 64 == 0)
                                puts("");
                        }
                        chnlpos = 193; // wait until next
                    }
                }
            }
                
            for (i = 0; i < 4; i++)
                subpkt[i][0] = subpkt[i][1] = ecc[i] = checkecc[i] = 0;

            pkthdr = ecchdr = checkecchdr= 0;
            bit = 0;

        }
    }
}

#define VGUARD_R 0x2CC
#define VGUARD_G 0x133
#define VGUARD_B 0x2CC

#define DGUARD_R 0x133
#define DGUARD_G 0x133

void display(int n, int red, int green, int blue, int clkbits, int dbgbits, int bits, int dbgpin)
{
    int terc_r, terc_g, terc_b;
    int i = 0;
    int h,v;
    int c_r,c_g,c_b;
    int decode = 0;

    terc_r = terc4_decode(red);
    terc_g = terc4_decode(green);
    terc_b = terc4_decode(blue);

    c_r = ctrl_decode(red);
    c_g = ctrl_decode(green);
    c_b = ctrl_decode(blue);

    if (n==1) // print heading once
        printf("    sample   bits      Type                             Red(ch2)  \tGreen(ch1)\tBlue(ch0)\t\tClock   %s\n",dbgpin >= 0 ? "\tDebug":"");
        
    i = printf("%10d ", n);
    i+=printf("%c %4d :->  ", bits == 10 ? ' ':'*', bits);
    if (c_b >= 0) // control
    {
        if (c_r == 0 && c_g == 1)
            i+=printf("%-20s", "VideoPreamble");
        else if (c_r == 1 && c_g == 1)
            i+=printf("%-20s", "DataPreamble    ");
        else 
            if (c_r == 0 && c_g == 0)
            i+=printf("Blanking  VH=%d      ", c_b);
        else
            i+=printf("Ctrl c32=%d c10=%d VH=%d ", c_r, c_g, c_b);
        i+= printf(" %s ", c_b & 2 ? "+V" : "V-");
        i+= printf(" %s ", c_b & 1 ? "+H" : "H-");
    } 
    else if (red == VGUARD_R && green == VGUARD_G && blue == VGUARD_B)
    {
        i+=printf("%-20s", "VideoGuard");
    }
    else if (red == DGUARD_R && green == DGUARD_G && terc_b >= 0xC)
    {
        i+=printf("%-20s", "DataGuard ");
        h = terc_b & 1;
        v = (terc_b & 2) >> 1;
        i+= printf(" %s ", v ? "+V" : "V-");
        i+= printf(" %s ", h ? "+H" : "H-");
    }
    else if (terc_r >= 0 && terc_g >= 0 && terc_b >= 0)
    {
        i+=printf("TERC4 ");
        i+=printbin(terc_r, 4);
        i+=printf("_");
        i+=printbin(terc_g, 4);
        i+=printf("_");
        i+=printbin(terc_b, 4);
        h = terc_b & 1;  // hsync
        v = (terc_b>>1) & 1; //vsync
        i+= printf(" %s ", v ? "+V" : "V-");
        i+= printf(" %s ", h ? "+H" : "H-");
        decode = 1;
    }
    else // not sure what else matches, so try to default to RGB
        i+=printf("RGB (%02X_%02X_%02X)  ", tmdsdecode(red), tmdsdecode(green), tmdsdecode(blue));
    
    while (i<56)
        i+=printf(" ");
    // show each in binary
    i+=printbin(red, bits);
    i+=printf("\t");
    i+=printbin(green, bits);
    i+=printf("\t");
    i+=printbin(blue, bits);
    i+=printf("\t\t");
    i+=printbin(clkbits, bits);
    i+=printf("\t");
    if (dbgpin>=0)
        i+=printbin(dbgbits, bits);
    i+=printf("\n");
    if (decode)
        decode_packet(terc_r, terc_g, terc_b);
}

int main(int argc, char **argv)
{
    int red, green, blue, clk;
    int r = 0, g = 0, b = 0;
    int ch;
    int oldclk = 0;
    int newclk;
    int count = 0;
    int bits = 0;
    int clkbits = 0;
    int dbgbits = 0;
    int dbgpin;
    int intmode = 0;
    unsigned int val = 0;

    if (argc==2 && strcmp(argv[1], "-l")==0)
        intmode = 1;
    else if (argc<5)
            usage();
    else
    {
        r = atoi(argv[1]);
        g = atoi(argv[2]);
        b = atoi(argv[3]);
        clk = atoi(argv[4]);
        if (argc > 5)
            dbgpin = atoi(argv[5]);
        else
            dbgpin = -1;
    }

    if (intmode)
        printf("Decoding TMDS bits on 10b triplets per int:\n");
    else
        printf("Decoding TMDS bits on binary input bytes:\n");
    while ((ch = getchar()) != EOF)
    {
        if (intmode)   
        {

            val = (ch << bits ) + val;
            bits += 8;
            if (bits == 32)
            {
                bits = 0;
                red = (val >> 22) & 0x3ff;
                green = (val >> 12) & 0x3ff;
                blue = (val >> 2 ) & 0x3ff;
                //printbin((int)val,32);
                //printf(" %08x  red=%03x  green=%03x  blue=%03x\n",val, red, green, blue);
                display(++count, red, green, blue, 0x1f, 0,10,-1);
                val =0;
            }
            continue;
        }
        newclk = (ch >> clk) & 1;
        if ((oldclk == 0) && newclk) 
        { // new sample starts now
            // display last sample
            count++;
            display(count, red, green, blue, clkbits, dbgbits, bits, dbgpin);
            red=green=blue=bits=clkbits=dbgbits=0;   
        } 
        red     |= ((ch >> r)      & 1) ? (1<<bits): 0;
        green   |= ((ch >> g)      & 1) ? (1<<bits): 0;
        blue    |= ((ch >> b)      & 1) ? (1<<bits): 0;
        clkbits |= ((ch >> clk)    & 1) ? (1<<bits): 0;
        if (dbgpin >= 0)
            dbgbits |= ((ch >> dbgpin) & 1) ? (1<<bits): 0;
        oldclk = newclk;
        bits++;
    }
    puts("Done");
    return 0;
}


