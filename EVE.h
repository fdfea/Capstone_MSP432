#ifndef __EVE_H
#define __EVE_H

#include <stdint.h>

#include <ti/drivers/SPI.h>
#include <ti/devices/msp432p4xx/inc/msp.h>

SPI_Handle      spiHandle;
SPI_Params      spiParams;
SPI_Transaction spiTransaction;

uint8_t *txBuf, *rxBuf;
int txIdx;

// Display Settings
#define PCLK		(2L)
#define PCLK_POL	(1L)
#define SWIZZLE		(0L)
#define CSPREAD		(0L)
#define HSIZE		(800L)	/* Thd Length of visible part of line (in PCLKs) - display width */
#define VSIZE		(480L)	/* Tvd Number of visible lines (in lines) - display height */
#define VSYNC0		(0L)	/* Tvf Vertical Front Porch */
#define VSYNC1		(3L)	/* Tvf + Tvp Vertical Front Porch plus Vsync Pulse width */
#define VOFFSET		(32L)	/* Tvf + Tvp + Tvb Number of non-visible lines (in lines) */
#define VCYCLE		(525L)	/* Tv Total number of lines (visible and non-visible) (in lines) */
#define HSYNC0		(0L)	 /* (40L)	// Thf Horizontal Front Porch */
#define HSYNC1		(48L)	/* Thf + Thp Horizontal Front Porch plus Hsync Pulse width */
#define HOFFSET		(88L)	/* Thf + Thp + Thb Length of non-visible part of line (in PCLK cycles) */
#define HCYCLE 		(928L)	/* Th Total length of line (visible and non-visible) (in PCLKs) */
#define DITHER      (1L)

// MSP Ports
#define EVE_PDN_PORT        P6
#define EVE_PDN             0x0004
#define EVE_CS_PORT         P9
#define EVE_CS              0x0010

#define WRITE               0x80
#define READ                0x00

#define ACTIVE              0x00
#define STANDBY             0x41
#define SLEEP               0x42
#define PWRDOWN             0x50
#define CLKINT              0x48
#define CLKEXT              0x44
#define CLK48M              0x62
#define CLK36M              0x61
#define CORERESET           0x68

#define DL_CLEAR      0x26000000 /* requires OR'd arguments */
#define DL_CLEAR_RGB  0x02000000 /* requires OR'd arguments */
#define DL_COLOR_RGB  0x04000000 /* requires OR'd arguments */
#define DL_POINT_SIZE 0x0D000000 /* requires OR'd arguments */
#define DL_END        0x21000000
#define DL_BEGIN      0x1F000000 /* requires OR'd arguments */
#define DL_DISPLAY    0x00000000

#define CLR_COL       0x4
#define CLR_STN       0x2
#define CLR_TAG       0x1

// Sound
#define SILENCE              0x00
#define SQUAREWAVE           0x01
#define SINEWAVE             0x02
#define SAWTOOTH             0x03
#define TRIANGLE             0x04
#define BEEPING              0x05
#define ALARM                0x06
#define WARBLE               0x07
#define CAROUSEL             0x08
#define PIPS(n)              (0x0F + (n))
#define HARP                 0x40
#define XYLOPHONE            0x41
#define TUBA                 0x42
#define GLOCKENSPIEL         0x43
#define ORGAN                0x44
#define TRUMPET              0x45
#define PIANO                0x46
#define CHIMES               0x47
#define MUSICBOX             0x48
#define BELL                 0x49
#define CLICK                0x50
#define SWITCH               0x51
#define COWBELL              0x52
#define NOTCH                0x53
#define HIHAT                0x54
#define KICKDRUM             0x55
#define POP                  0x56
#define CLACK                0x57
#define CHACK                0x58
#define MUTE                 0x60
#define UNMUTE               0x61


// Pitch
#define MIDI_A0   21
#define MIDI_A_0  22
#define MIDI_B0   23
#define MIDI_C1   24
#define MIDI_C_1  25
#define MIDI_D1   26
#define MIDI_D_1  27
#define MIDI_E1   28
#define MIDI_F1   29
#define MIDI_F_1  30
#define MIDI_G1   31
#define MIDI_G_1  32
#define MIDI_A1   33
#define MIDI_A_1  34
#define MIDI_B1   35
#define MIDI_C2   36
#define MIDI_C_2  37
#define MIDI_D2   38
#define MIDI_D_2  39
#define MIDI_E2   40
#define MIDI_F2   41
#define MIDI_F_2  42
#define MIDI_G2   43
#define MIDI_G_2  44
#define MIDI_A2   45
#define MIDI_A_2  46
#define MIDI_B2   47
#define MIDI_C3   48
#define MIDI_C_3  49
#define MIDI_D3   50
#define MIDI_D_3  51
#define MIDI_E3   52
#define MIDI_F3   53
#define MIDI_F_3  54
#define MIDI_G3   55
#define MIDI_G_3  56
#define MIDI_A3   57
#define MIDI_A_3  58
#define MIDI_B3   59
#define MIDI_C4   60
#define MIDI_C_4  61
#define MIDI_D4   62
#define MIDI_D_4  63
#define MIDI_E4   64
#define MIDI_F4   65
#define MIDI_F_4  66
#define MIDI_G4   67
#define MIDI_G_4  68
#define MIDI_A4   69
#define MIDI_A_4  70
#define MIDI_B4   71
#define MIDI_C5   72
#define MIDI_C_5  73
#define MIDI_D5   74
#define MIDI_D_5  75
#define MIDI_E5   76
#define MIDI_F5   77
#define MIDI_F_5  78
#define MIDI_G5   79
#define MIDI_G_5  80
#define MIDI_A5   81
#define MIDI_A_5  82
#define MIDI_B5   83
#define MIDI_C6   84
#define MIDI_C_6  85
#define MIDI_D6   86
#define MIDI_D_6  87
#define MIDI_E6   88
#define MIDI_F6   89
#define MIDI_F_6  90
#define MIDI_G6   91
#define MIDI_G_6  92
#define MIDI_A6   93
#define MIDI_A_6  94
#define MIDI_B6   95
#define MIDI_C7   96
#define MIDI_C_7  97
#define MIDI_D7   98
#define MIDI_D_7  99
#define MIDI_E7   100
#define MIDI_F7   101
#define MIDI_F_7  102
#define MIDI_G7   103
#define MIDI_G_7  104
#define MIDI_A7   105
#define MIDI_A_7  106
#define MIDI_B7   107
#define MIDI_C8   108


// Commands
#define CMD_APPEND       0xFFFFFF1E
#define CMD_BGCOLOR      0xFFFFFF09
#define CMD_BUTTON       0xFFFFFF0D
#define CMD_CALIBRATE    0xFFFFFF15
#define CMD_CLOCK        0xFFFFFF14
#define CMD_COLDSTART    0xFFFFFF32
#define CMD_DIAL         0xFFFFFF2D
#define CMD_DLSTART      0xFFFFFF00
#define CMD_FGCOLOR      0xFFFFFF0A
#define CMD_GAUGE        0xFFFFFF13
#define CMD_GETMATRIX    0xFFFFFF33
#define CMD_GETPROPS     0xFFFFFF25
#define CMD_GETPTR       0xFFFFFF23
#define CMD_GRADCOLOR    0xFFFFFF34
#define CMD_GRADIENT     0xFFFFFF0B
#define CMD_INFLATE      0xFFFFFF22
#define CMD_INTERRUPT    0xFFFFFF02
#define CMD_KEYS         0xFFFFFF0E
#define CMD_LOADIDENTITY 0xFFFFFF26
#define CMD_LOADIMAGE    0xFFFFFF24
#define CMD_LOGO         0xFFFFFF31
#define CMD_MEDIAFIFO    0xFFFFFF39
#define CMD_MEMCPY       0xFFFFFF1D
#define CMD_MEMCRC       0xFFFFFF18
#define CMD_MEMSET       0xFFFFFF1B
#define CMD_MEMWRITE     0xFFFFFF1A
#define CMD_MEMZERO      0xFFFFFF1C
#define CMD_NUMBER       0xFFFFFF2E
#define CMD_PLAYVIDEO    0xFFFFFF3A
#define CMD_PROGRESS     0xFFFFFF0F
#define CMD_REGREAD      0xFFFFFF19
#define CMD_ROMFONT      0xFFFFFF3F
#define CMD_ROTATE       0xFFFFFF29
#define CMD_SCALE        0xFFFFFF28
#define CMD_SCREENSAVER  0xFFFFFF2F
#define CMD_SCROLLBAR    0xFFFFFF11
#define CMD_SETBASE      0xFFFFFF38
#define CMD_SETBITMAP    0xFFFFFF43
#define CMD_SETFONT      0xFFFFFF2B
#define CMD_SETFONT2     0xFFFFFF3B
#define CMD_SETMATRIX    0xFFFFFF2A
#define CMD_SETROTATE    0xFFFFFF36
#define CMD_SETSCRATCH   0xFFFFFF3C
#define CMD_SKETCH       0xFFFFFF30
#define CMD_SLIDER       0xFFFFFF10
#define CMD_SNAPSHOT     0xFFFFFF1F
#define CMD_SNAPSHOT2    0xFFFFFF37
#define CMD_SPINNER      0xFFFFFF16
#define CMD_STOP         0xFFFFFF17
#define CMD_SWAP         0xFFFFFF01
#define CMD_TEXT         0xFFFFFF0C
#define CMD_TOGGLE       0xFFFFFF12
#define CMD_TRACK        0xFFFFFF2C
#define CMD_TRANSLATE    0xFFFFFF27
#define CMD_VIDEOFRAME   0xFFFFFF41
#define CMD_VIDEOSTART   0xFFFFFF40

// BT81X COMMANDS 
#define CMD_BITMAP_TRANSFORM 0xFFFFFF21
#define CMD_SYNC             0xFFFFFF42		
#define CMD_FLASHERASE       0xFFFFFF44		
#define CMD_FLASHWRITE       0xFFFFFF45
#define CMD_FLASHREAD        0xFFFFFF46
#define CMD_FLASHUPDATE      0xFFFFFF47
#define CMD_FLASHDETACH      0xFFFFFF48		
#define CMD_FLASHATTACH      0xFFFFFF49		
#define CMD_FLASHFAST        0xFFFFFF4A
#define CMD_FLASHSPIDESEL    0xFFFFFF4B		
#define CMD_FLASHSPITX       0xFFFFFF4C
#define CMD_FLASHSPIRX       0xFFFFFF4D
#define CMD_FLASHSOURCE      0xFFFFFF4E
#define CMD_CLEARCACHE       0xFFFFFF4F		
#define CMD_INFLATE2         0xFFFFFF50
#define CMD_ROTATEAROUND     0xFFFFFF51
#define CMD_RESETFONTS       0xFFFFFF52		
#define CMD_ANIMSTART        0xFFFFFF53
#define CMD_ANIMSTOP         0xFFFFFF54
#define CMD_ANIMXY           0xFFFFFF55
#define CMD_ANIMDRAW         0xFFFFFF56
#define CMD_GRADIENTA        0xFFFFFF57
#define CMD_FILLWIDTH        0xFFFFFF58
#define CMD_APPENDF          0xFFFFFF59
#define CMD_ANIMFRAME        0xFFFFFF5A
#define CMD_VIDEOSTARTF      0xFFFFFF5F

#define DLSWAP_FRAME         2UL

#define OPT_CENTER           1536UL
#define OPT_CENTERX          512UL
#define OPT_CENTERY          1024UL
#define OPT_FLASH            64UL
#define OPT_FLAT             256UL
#define OPT_FULLSCREEN       8UL
#define OPT_MEDIAFIFO        16UL
#define OPT_MONO             1UL
#define OPT_NOBACK           4096UL
#define OPT_NODL             2UL
#define OPT_NOHANDS          49152UL
#define OPT_NOHM             16384UL
#define OPT_NOPOINTER        16384UL
#define OPT_NOSECS           32768UL
#define OPT_NOTEAR           4UL
#define OPT_NOTICKS          8192UL
#define OPT_RGB565           0UL
#define OPT_RIGHTX           2048UL
#define OPT_SIGNED           256UL
#define OPT_SOUND            32UL
#define OPT_FILL   			 8192UL

// Definitions for FT8xx co processor command buffer
#define FT_DL_SIZE           (8*1024)  // 8KB Display List buffer size
#define FT_CMD_FIFO_SIZE     (4*1024)  // 4KB coprocessor Fifo size
#define FT_CMD_SIZE          (4)       // 4 byte per coprocessor command of EVE

// Memory base addresses
#define RAM_G                    0x0
#define RAM_G_WORKING            0x0FF000 // This address may be used as the start of a 4K block to be used for copying data
#define RAM_DL                   0x300000
#define RAM_REG                  0x302000
#define RAM_CMD                  0x308000
#define RAM_ERR_REPORT           0x309800 // max 128 bytes null terminated string
#define RAM_FLASH                0x800000
#define RAM_FLASH_POSTBLOB       0x801000

// Graphics Engine Registers - FT81x Series Programmers Guide Section 3.1
#define REG_CSPREAD               0x68
#define REG_DITHER                0x60
#define REG_DLSWAP                0x54
#define REG_HCYCLE                0x2C
#define REG_HOFFSET               0x30    
#define REG_HSIZE                 0x34
#define REG_HSYNC0                0x38
#define REG_HSYNC1                0x3C
#define REG_OUTBITS               0x5C
#define REG_PCLK                  0x70
#define REG_PCLK_POL              0x6C
#define REG_PLAY                  0x8C
#define REG_PLAYBACK_FORMAT       0xC4
#define REG_PLAYBACK_FREQ         0xC0
#define REG_PLAYBACK_LENGTH       0xB8
#define REG_PLAYBACK_LOOP         0xC8
#define REG_PLAYBACK_PLAY         0xCC
#define REG_PLAYBACK_READPTR      0xBC
#define REG_PLAYBACK_START        0xB4
#define REG_PWM_DUTY              0xD4
#define REG_ROTATE                0x58
#define REG_SOUND                 0x88
#define REG_SWIZZLE               0x64
#define REG_TAG                   0x7C
#define REG_TAG_X                 0x74
#define REG_TAG_Y                 0x78
#define REG_VCYCLE                0x40
#define REG_VOFFSET               0x44
#define REG_VOL_SOUND             0x84
#define REG_VOL_PB                0x80
#define REG_VSYNC0                0x4C
#define REG_VSYNC1                0x50
#define REG_VSIZE                 0x48 

// Co-processor Engine Registers - FT81x Series Programmers Guide Section 3.4
// Addresses defined as offsets from the base address called RAM_REG and located at 0x302000
#define REG_CMD_DL                0x100
#define REG_CMD_READ              0xF8
#define REG_CMD_WRITE             0xFC
#define REG_CMDB_SPACE            0x574
#define REG_CMDB_WRITE            0x578
#define REG_COPRO_PATCH_PTR       0x7162

// Special Registers - FT81x Series Programmers Guide Section 3.5 
// Addresses assumed to be defined as offsets from the base address called RAM_REG and located at 0x302000
#define REG_TRACKER               0x7000
#define REG_TRACKER_1             0x7004
#define REG_TRACKER_2             0x7008
#define REG_TRACKER_3             0x700C
#define REG_TRACKER_4             0x7010
#define REG_MEDIAFIFO_READ        0x7014
#define REG_MEDIAFIFO_WRITE       0x7018

// Flash related registers
#define REG_FLASH_STATUS          0x5F0
#define REG_FLASH_SIZE            0x7024

// Miscellaneous Registers - FT81x Series Programmers Guide Section 3.6 - Document inspecific about base address
// Addresses assumed to be defined as offsets from the base address called RAM_REG and located at 0x302000
#define REG_CPU_RESET             0x20
#define REG_PWM_DUTY              0xD4
#define REG_PWM_HZ                0xD0
#define REG_INT_MASK              0xB0
#define REG_INT_EN                0xAC
#define REG_INT_FLAGS             0xA8
#define REG_GPIO                  0x94
#define REG_GPIO_DIR              0x90
#define REG_GPIOX                 0x9C
#define REG_GPIOX_DIR             0x98
#define REG_FREQUENCY             0x0C
#define REG_CLOCK                 0x08
#define REG_FRAMES                0x04
#define REG_ID                    0x00
#define REG_TRIM                  0x10256C
#define REG_SPI_WIDTH             0x180
#define REG_CHIP_ID               0xC0000   // Temporary Chip ID location in RAMG

// Primitive Type Reference Definitions - FT81x Series Programmers Guide Section 4.5 - Table 6
#define BITMAPS                    1
#define POINTS                     2
#define LINES                      3
#define LINE_STRIP                 4
#define EDGE_STRIP_R               5
#define EDGE_STRIP_L               6
#define EDGE_STRIP_A               7
#define EDGE_STRIP_B               8
#define RECTS                      9

// Bitmap Layout Format Definitions - FT81x Series Programmers Guide Section 4.7 - Table 7
#define ARGB1555                           0
#define L1                                 1
#define L4                                 2
#define L8                                 3
#define RGB332                             4
#define ARGB2                              5
#define ARGB4                              6
#define RGB565                             7
#define TEXT8X8                            9
#define TEXTVGA                           10
#define BARGRAPH                          11
#define PALETTED565                       14
#define PALETTED4444                      15
#define PALETTED8                         16
#define L2                                17

// Bitmap Layout Format Definitions - BT81X Series Programming Guide Section 4.6
#define COMPRESSED_RGBA_ASTC_4x4_KHR   37808  // 8.00
#define COMPRESSED_RGBA_ASTC_5x4_KHR   37809  // 6.40
#define COMPRESSED_RGBA_ASTC_5x5_KHR   37810  // 5.12
#define COMPRESSED_RGBA_ASTC_6x5_KHR   37811  // 4.27
#define COMPRESSED_RGBA_ASTC_6x6_KHR   37812  // 3.56
#define COMPRESSED_RGBA_ASTC_8x5_KHR   37813  // 3.20
#define COMPRESSED_RGBA_ASTC_8x6_KHR   37814  // 2.67
#define COMPRESSED_RGBA_ASTC_8x8_KHR   37815  // 2.56
#define COMPRESSED_RGBA_ASTC_10x5_KHR  37816  // 2.13
#define COMPRESSED_RGBA_ASTC_10x6_KHR  37817  // 2.00
#define COMPRESSED_RGBA_ASTC_10x8_KHR  37818  // 1.60
#define COMPRESSED_RGBA_ASTC_10x10_KHR 37819  // 1.28
#define COMPRESSED_RGBA_ASTC_12x10_KHR 37820  // 1.07
#define COMPRESSED_RGBA_ASTC_12x12_KHR 37821  // 0.89

// Bitmap Parameters
#define REPEAT                     1
#define BORDER                     0
#define NEAREST                    0
#define BILINEAR                   1

// Flash Status
#define FLASH_STATUS_INIT          0UL
#define FLASH_STATUS_DETACHED      1UL
#define FLASH_STATUS_BASIC         2UL
#define FLASH_STATUS_FULL          3UL


// These defined "macros" are supplied by FTDI - Manufacture command bit-fields from parameters
// FT81x Series Programmers Guide is refered to as "FT-PG"
#define CLEAR(c,s,t) ((38UL<<24)|(((c)&1UL)<<2)|(((s)&1UL)<<1)|(((t)&1UL)<<0))                                                                                           // CLEAR - FT-PG Section 4.21
#define CLEAR_COLOR_RGB(red,green,blue) ((2UL<<24)|(((red)&255UL)<<16)|(((green)&255UL)<<8)|(((blue)&255UL)<<0))                                                         // CLEAR_COLOR_RGB - FT-PG Section 4.23
#define COLOR_RGB(red,green,blue) ((4UL<<24)|(((red)&255UL)<<16)|(((green)&255UL)<<8)|(((blue)&255UL)<<0))                                                               // COLOR_RGB - FT-PG Section 4.28
#define VERTEX2II(x,y,handle,cell) ((2UL<<30)|(((x)&511UL)<<21)|(((y)&511UL)<<12)|(((handle)&31UL)<<7)|(((cell)&127UL)<<0))                                              // VERTEX2II - FT-PG Section 4.48
#define VERTEX2F(x,y) ((1UL<<30)|(((x)&32767UL)<<15)|(((y)&32767UL)<<0))                                                                                                 // VERTEX2F - FT-PG Section 4.47
#define CELL(cell) ((6UL<<24)|(((cell)&127UL)<<0))                                                                                                                       // CELL - FT-PG Section 4.20
#define BITMAP_HANDLE(handle) ((5UL<<24) | (((handle) & 31UL) << 0))                                                                                                     // BITMAP_HANDLE - FT-PG Section 4.06
#define BITMAP_SOURCE(addr) ((1UL<<24)|(((addr)&1048575UL)<<0))                                                                                                          // BITMAP_SOURCE - FT-PG Section 4.11
#define BITMAP_LAYOUT(format,linestride,height) ((7UL<<24)|(((format)&31UL)<<19)|(((linestride)&1023UL)<<9)|(((height)&511UL)<<0))                                       // BITMAP_LAYOUT - FT-PG Section 4.07
#define BITMAP_SIZE(filter,wrapx,wrapy,width,height) ((8UL<<24)|(((filter)&1UL)<<20)|(((wrapx)&1UL)<<19)|(((wrapy)&1UL)<<18)|(((width)&511UL)<<9)|(((height)&511UL)<<0)) // BITMAP_SIZE - FT-PG Section 4.09
#define TAG(s) ((3UL<<24)|(((s)&255UL)<<0))                                                                                                                              // TAG - FT-PG Section 4.43
#define POINT_SIZE(sighs) ((13UL<<24)|(((sighs)&8191UL)<<0))                                                                                                             // POINT_SIZE - FT-PG Section 4.36
#define BEGIN(PrimitiveTypeRef) ((31UL<<24)|(((PrimitiveTypeRef)&15UL)<<0))                                                                                              // BEGIN - FT-PG Section 4.05
#define END() ((33UL<<24))                                                                                                                                               // END - FT-PG Section 4.30
#define DISPLAY() ((0UL<<24))                                                                                                                                            // DISPLAY - FT-PG Section 4.29

// Non FTDI Helper Macros
#define MAKE_COLOR(r,g,b) (( r << 16) | ( g << 8) | (b))
#define LINE_WIDTH(width) ((14UL<<24)|(((width)&4095UL)<<0))
#define VERTEX_FORMAT(frac) ((39UL<<24)|(((frac)&7UL)<<0))



// Function Prototypes
void delay(uint32_t ms);
static inline void waitFIFO();
void EVE_reset();
int EVE_startBurst();
int EVE_sendBurst();
void EVE_burst8(uint8_t data);
void EVE_burst16(uint16_t data);
void EVE_burst32(uint32_t data);
void EVE_write8(uint32_t address, uint8_t data);
void EVE_write16(uint32_t address, uint16_t data);
void EVE_write32(uint32_t address, uint32_t data);
uint8_t EVE_read8(uint32_t address);
uint16_t EVE_read16(uint32_t address);
uint32_t EVE_read32(uint32_t address);
void EVE_sendHCMD(uint8_t command, uint8_t param);
void EVE_init();
uint32_t EVE_initFlash();
void EVE_setSound(uint16_t sound, uint16_t pitch);
void EVE_startSound();
void EVE_stopSound();
void EVE_toggleSound();
void EVE_setVolume(uint8_t vol);
void EVE_writeString(char* string);
void EVE_cmdFillWidth(uint32_t val);
void EVE_cmdBGColor(uint32_t color);
void EVE_cmdFGColor(uint32_t color);
void EVE_cmdROMFont(uint32_t font, uint32_t slot);
void EVE_cmdText(int16_t x, int16_t y, int16_t font, uint16_t options, char* text);
void EVE_cmdAppend(uint32_t ptr, uint32_t len);
void EVE_cmdSpinner(int16_t x, int16_t y, uint16_t style, uint16_t scale);
void EVE_cmdLoadImage(uint32_t dest, uint32_t options, uint8_t *data, uint32_t len);
void EVE_cmdSetBitmap(uint32_t address, uint16_t format, uint16_t width, uint16_t height);
void EVE_cmd(uint32_t cmd);
void EVE_colorRGB(uint32_t color);


#endif
