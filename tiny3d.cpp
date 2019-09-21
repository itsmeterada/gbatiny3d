//
//  3D Wireframe drawing sample for GBA
//	Takehiko Terada (its_me_terada@me.com)
//	2016/05/05
//

// 0x00000000 - 0x00003FFF – 16KBシステムROM（実行可能だが読み出し不可）
// 0x02000000 - 0x02030000 – 256 KB EWRAM (標準目的の外部RAM）
// 0x03000000 - 0x03007FFF – 32 KB IWRAM（標準目的の内部RAM）
// 0x04000000 - 0x040003FF – I/Oレジスタ
// 0x05000000 - 0x050003FF – 1KB カラーパレット RAM
// 0x06000000 - 0x06017FFF – 96KB VRAM （ビデオRAM）
// 0x07000000 - 0x070003FF – 1KB OAM RAM （オブジェクト属性メモリ – 後で説明します）
// 0x08000000 - 0x???????? – Game Pak ROM (0 から 32MB)
// 0x0E000000 - 0x???????? – Game Pak RAM

#include <stdlib.h>

//---------------------------------------------------------------------------------
// Data types
//---------------------------------------------------------------------------------
typedef unsigned char           u8;     /**< Unsigned 8 bit value       */
typedef unsigned short int      u16;    /**< Unsigned 16 bit value      */
typedef unsigned int            u32;    /**< Unsigned 32 bit value      */
typedef signed char             s8;     /**< Signed 8 bit value */
typedef signed short int        s16;    /**< Signed 16 bit value        */
typedef signed int              s32;    /**< Signed 32 bit value        */
typedef volatile u8             vu8;    /**< volatile Unsigned 8 bit value      */
typedef volatile u16            vu16;   /**< volatile Unigned 16 bit value      */
typedef volatile u32            vu32;   /**< volatile Unsigned 32 bit value     */
typedef volatile s8             vs8;    /**< volatile Signed 8 bit value        */
typedef volatile s16            vs16;   /**< volatile Signed 8 bit value        */
typedef volatile s32            vs32;   /**< volatile Signed 8 bit value        */
#ifndef __cplusplus
/*! C++ compatible bool for C */
typedef enum { false, true } bool;
#endif


#define ALIGN(p)	__attribute__((aligned(p)))

#define	REG_BASE				0x04000000

#define REG_DISPCNT		(vu16 *)REG_BASE
#define REG_VCOUNT		(vu16 *)0x04000006
#define REG_DISPSTAT	(vu16 *)0x04000004

vu16 *scanlineCounter = REG_VCOUNT;

//#define MODE5
#ifdef MODE5
#define DISP_MODE5	0x05
#define WIDTH 160
#define HEIGHT 128
#else // MODE3
#define DISP_MODE3	0x03
#define WIDTH 240
#define HEIGHT 160
#endif // MODE5

#define DISP_ENABLE_BG2	0x400

#define FIXED_BITS	8
#define FIXED_ONE	0x100
#define FIXED_HALF	0x80

// Background rotation / scale
#define REG_BG2PA		(vu16 *)0x04000020
#define REG_BG2PB		(vu16 *)0x04000022
#define REG_BG2PC		(vu16 *)0x04000024
#define REG_BG2PD		(vu16 *)0x04000026
#define REG_BG2X		(vu32 *)0x04000028
#define REG_BG2Y		(vu32 *)0x0400002C

#define REG_BG2CNT		(vu16 *)0x0400000C

// Timer data addresses
#define REG_TM0D				(vu16 *)0x04000100
#define REG_TM1D				(vu16 *)0x04000104
#define REG_TM2D				(vu16 *)0x04000108
#define REG_TM3D				(vu16 *)0x0400010C
// Timer status addresses
#define REG_TM0CNT				(vu16 *)0x04000102
#define REG_TM1CNT				(vu16 *)0x04000106
#define REG_TM2CNT				(vu16 *)0x0400010A
#define REG_TM3CNT				(vu16 *)0x0400010E
// Timer constants
#define TIMER_FREQUENCY_SYSTEM	0x00
#define TIMER_FREQUENCY_64		0x01
#define TIMER_FREQUENCY_256		0x02
#define TIMER_FREQUENCY_1024	0x03
#define TIMER_OVERFLOW			0x04
#define TIMER_IRQ_ENABLE		0x40
#define TIMER_ENABLE			0x80


// DMA
#define REG_DMA0SAD	*(vu32*)(REG_BASE + 0x0b0)
#define REG_DMA0DAD	*(vu32*)(REG_BASE + 0x0b4)
#define REG_DMA0CNT	*(vu32*)(REG_BASE + 0x0b8)
//--
#define REG_DMA1SAD	*(vu32*)(REG_BASE + 0x0bc)
#define REG_DMA1DAD	*(vu32*)(REG_BASE + 0x0c0)
#define REG_DMA1CNT	*(vu32*)(REG_BASE + 0x0c4)
//--
#define REG_DMA2SAD	*(vu32*)(REG_BASE + 0x0c8)
#define REG_DMA2DAD	*(vu32*)(REG_BASE + 0x0cc)
#define REG_DMA2CNT	*(vu32*)(REG_BASE + 0x0d0)
//--
#define REG_DMA3SAD	*(vu32*)(REG_BASE + 0x0d4)
#define REG_DMA3DAD	*(vu32*)(REG_BASE + 0x0d8)
#define REG_DMA3CNT	*(vu32*)(REG_BASE + 0x0dc)
//--
#define DMA_DST_INC		(0<<21)
#define DMA_DST_DEC		(1<<21)
#define DMA_DST_FIXED	(2<<21) // 固定
#define DMA_DST_RELOAD	(3<<21)

#define DMA_SRC_INC		(0<<23)
#define DMA_SRC_DEC		(1<<23)
#define DMA_SRC_FIXED	(2<<23) // 固定

#define DMA_REPEAT		(1<<25)
#define DMA_IRQ			(1<<30)
#define DMA_ENABLE		(1<<31)
#define DMA_IMMEDIATE	(0<<28)
#define DMA_VBLANK		(1<<28)
#define DMA_HBLANK		(2<<28)
#define DMA_SPECIAL		(3<<28)
#define DMA_16			0x00000000
#define DMA_32			0x04000000

#ifdef DEBUG
#define CODE_IN_IWRAM
#else // DEBUG
#define CODE_IN_IWRAM __attribute__ ((section (".iwram"), long_call))
#endif // DEBUG

vu16 *vram;
#define VRAM_ADRS		(vu16 *)0x06000000
#define RGB(r,g,b) ((((b)>>3)<<10)+(((g)>>3)<<5)+((r)>>3))
//u16 surface[WIDTH * HEIGHT] ALIGN(4);
u16 clearColor ALIGN(4);
//u8 *buffer;
u16 *buffer;
#define drawPixel(x, y, c)	(buffer[y*WIDTH+x] = c)
#define SETPIXEL(x, y, c)	(buffer[y*WIDTH+x] = c)
void drawLineEFLA(int x1, int y1, int x2, int y2, unsigned short color);

volatile u32 *BUTTONS = (volatile u32*)0x04000130;
#define BUTTON_A		1
#define BUTTON_B		2
#define BUTTON_SELECT	4
#define BUTTON_START	8
#define BUTTON_RIGHT	16
#define BUTTON_LEFT		32
#define BUTTON_UP		64
#define BUTTON_DOWN		128
#define BUTTON_R		256
#define BUTTON_L		512

bool buttons[10];

#define NUM_POINTS		200

#include "type.h"
#include "mesh_torus.h"

#define SCREENW   240
#define SCREENH   160
#define HALFW   120
#define HALFH   80
#define FOV     64


Matrix4 m_world;
Vector3i mesh_rotation = {0, 0, 0};
Vector3i mesh_position = {0, 0, 0};

const UINT16 lut[] = {         // 0 to 90 degrees fixed point COSINE look up table
  16384, 16381, 16374, 16361, 16344, 16321, 16294, 16261, 16224, 16182, 16135, 16082, 16025, 15964, 15897, 15825, 15749, 15668, 15582, 15491, 15395, 15295, 15190, 15081, 14967, 14848, 14725, 14598, 14466, 14329, 14188, 14043, 13894, 13740, 13582, 13420, 13254, 13084, 12910, 12732, 12550, 12365, 12175, 11982, 11785, 11585, 11381, 11173, 10963, 10748, 10531, 10310, 10086, 9860, 9630, 9397, 9161, 8923, 8682, 8438, 8191, 7943, 7691, 7438, 7182, 6924, 6663, 6401, 6137, 5871, 5603, 5334, 5062, 4790, 4516, 4240, 3963, 3685, 3406, 3126, 2845, 2563, 2280, 1996, 1712, 1427, 1142, 857, 571, 285, 0
};

#define LUT(a)  (UINT32)(lut[a])

static INT16 proj_nodes[NODECOUNT][2];         // projected nodes (x,y)
static INT16 old_nodes[NODECOUNT][2];          // projected nodes of previous frame to check if we need to redraw
static unsigned char i;

static unsigned char draw_type = 0;          // 0 - vertex | 1 - wireframe | 2 - flat colors | ...

// ----------------------------------------------
// SIN/COS from 90 degrees LUT
// ----------------------------------------------
INT32 CODE_IN_IWRAM SIN(UINT16 angle) {
  angle += 90;
  if (angle > 450) return LUT(0);
  if (angle > 360 && angle < 451) return -LUT(angle-360);
  if (angle > 270 && angle < 361) return -LUT(360-angle);
  if (angle > 180 && angle < 271) return  LUT(angle-180);
  return LUT(180-angle);
}

INT32 CODE_IN_IWRAM COS(UINT16 angle) {
  if (angle > 360) return LUT(0);
  if (angle > 270 && angle < 361) return  LUT(360-angle);
  if (angle > 180 && angle < 271) return -LUT(angle-180);
  if (angle > 90  && angle < 181) return -LUT(180-angle);
  return LUT(angle);
}

// ----------------------------------------------
// Matrix operation
// ----------------------------------------------
Matrix4 CODE_IN_IWRAM mMultiply(const Matrix4 &mat1, const Matrix4 &mat2)
{
  Matrix4 mat;
  unsigned char r,c;
  for (c=0; c<4; c++)
    for (r=0; r<4; r++)
      mat.m[c][r] = pMultiply(mat1.m[0][r], mat2.m[c][0]) +
                    pMultiply(mat1.m[1][r], mat2.m[c][1]) +
                    pMultiply(mat1.m[2][r], mat2.m[c][2]) +
                    pMultiply(mat1.m[3][r], mat2.m[c][3]);
  return mat;
}

Matrix4 CODE_IN_IWRAM mRotateX(const UINT16 angle)
{
  Matrix4 mat;
  mat.m[1][1] =  COS(angle);
  mat.m[1][2] =  SIN(angle);
  mat.m[2][1] = -SIN(angle);
  mat.m[2][2] =  COS(angle);
  return mat;
}

Matrix4 CODE_IN_IWRAM mRotateY(const UINT16 angle)
{
  Matrix4 mat;
  mat.m[0][0] =  COS(angle);
  mat.m[0][2] = -SIN(angle);
  mat.m[2][0] =  SIN(angle);
  mat.m[2][2] =  COS(angle);
  return mat;
}

Matrix4 CODE_IN_IWRAM mRotateZ(const UINT16 angle)
{
  Matrix4 mat;
  mat.m[0][0] =  COS(angle);
  mat.m[0][1] =  SIN(angle);
  mat.m[1][0] = -SIN(angle);
  mat.m[1][1] =  COS(angle);
  return mat;
}

Matrix4 CODE_IN_IWRAM mTranslate(const INT32 x, const INT32 y, const INT32 z)
{
  Matrix4 mat;
  mat.m[3][0] =  x << PSHIFT;
  mat.m[3][1] =  y << PSHIFT;
  mat.m[3][2] =  z << PSHIFT;
  return mat;
}

Matrix4 CODE_IN_IWRAM mScale(const float ratio)
{
  Matrix4 mat;
  mat.m[0][0] *= ratio;
  mat.m[1][1] *= ratio;
  mat.m[2][2] *= ratio;
  return mat;
}

// ----------------------------------------------
// Shoelace algorithm to get the surface
// ----------------------------------------------
int CODE_IN_IWRAM shoelace(const INT16 (*n)[2], const unsigned char index)
{
  unsigned char t = 0;
  INT16 surface = 0;
  for (; t<3; t++) {
    // (x1y2 - y1x2) + (x2y3 - y2x3) ...
    surface += (n[EDGE(index,t)][0]           * n[EDGE(index,(t<2?t+1:0))][1]) -
               (n[EDGE(index,(t<2?t+1:0))][0] * n[EDGE(index,t)][1]);
  }
  return surface * 0.5;
}

// ----------------------------------------------
// Shoelace algorithm for triangle visibility
// ----------------------------------------------
bool CODE_IN_IWRAM is_hidden(const INT16 (*n)[2], const unsigned char index)
{
  // (x1y2 - y1x2) + (x2y3 - y2x3) ...
  return ( ( (n[EDGE(index,0)][0] * n[EDGE(index,1)][1]) -
             (n[EDGE(index,1)][0] * n[EDGE(index,0)][1])   ) +
           ( (n[EDGE(index,1)][0] * n[EDGE(index,2)][1]) -
             (n[EDGE(index,2)][0] * n[EDGE(index,1)][1])   ) +
           ( (n[EDGE(index,2)][0] * n[EDGE(index,0)][1]) -
             (n[EDGE(index,0)][0] * n[EDGE(index,2)][1])   ) ) < 0 ? false : true;
}

// ----------------------------------------------
// draw projected nodes
// ----------------------------------------------
void CODE_IN_IWRAM draw_vertex(const INT16 (*n)[2], const UINT16 color)
{
  i = NODECOUNT-1;
  do {
    drawPixel(n[i][0],n[i][1], color);
  } while(i--);
}

// ----------------------------------------------
// draw edges between projected nodes
// ----------------------------------------------
void CODE_IN_IWRAM draw_wireframe(const INT16 (*n)[2], const UINT16 color)
{
  i = TRICOUNT-1;
  do {
    // don't draw triangle with negative surface value
    if (!is_hidden(n, i)) {
      // draw triangle edges - 0 -> 1 -> 2 -> 0
      drawLineEFLA(n[EDGE(i,0)][0], n[EDGE(i,0)][1], n[EDGE(i,1)][0], n[EDGE(i,1)][1], color);
      drawLineEFLA(n[EDGE(i,1)][0], n[EDGE(i,1)][1], n[EDGE(i,2)][0], n[EDGE(i,2)][1], color);
      drawLineEFLA(n[EDGE(i,2)][0], n[EDGE(i,2)][1], n[EDGE(i,0)][0], n[EDGE(i,0)][1], color);
    }
  } while(i--);
}


// Simple random number generator.
// -- Start --
unsigned int _num = 0;

void CODE_IN_IWRAM xorshiftSeed(unsigned int seed)
{
	_num = seed;
}

unsigned int CODE_IN_IWRAM xorshift32()
{
	_num = _num ^ (_num << 13);
	_num = _num ^ (_num >> 17);
	_num = _num ^ (_num << 15);
	return _num;
}
// -- End --

// Not working ?
void CODE_IN_IWRAM waitForVSync()
{
	while((*REG_DISPSTAT & 1));
}

void CODE_IN_IWRAM waitForVBlank()
{
#if 1
	while(!(*scanlineCounter));
	while(*scanlineCounter);
#else
    __asm
    {
     mov r0,#0x04000004
     wait_retrace:
       ldrh r1,[r0]
       tst r1,#1
     bne wait_retrace
    }
#endif
}


void CODE_IN_IWRAM *mallocAligned(u32 size, int align)
{
	void *p = malloc(size + align);
	if ((u32)p % align) p += ((u32)p % align);
	return p;
}

void CODE_IN_IWRAM
drawLineEFLA(int x1, int y1, int x2, int y2, unsigned short color)
{
    bool yLonger = false;
    int shortLen = y2 - y1;
    int longLen = x2 - x1;
    int decInc;
#if 1
    if ((shortLen ^ (shortLen >> 31)) - (shortLen >> 31) > (longLen ^ (longLen >> 31)) - (longLen >> 31)) {
        shortLen ^= longLen;
        longLen ^= shortLen;
        shortLen ^= longLen;
        yLonger = true;
    }
#else
    if (abs(shortLen) > abs(longLen)) {
        int swap = shortLen;
        shortLen = longLen;
        longLen = swap;
        yLonger = true;
    }
#endif
    if (longLen == 0) decInc = 0;
    else decInc = (shortLen << 16) / longLen;

    if (yLonger) {
        if (longLen > 0) {
            longLen += y1;
            for (int j = 0x8000 + (x1 << 16); y1 <= longLen; ++y1) {
                SETPIXEL((j >> 16), y1, color);
                j+= decInc;
            }
            return;
        }
        longLen += y1;
        for (int j = 0x8000 + (x1 << 16); y1 >= longLen; --y1) {
            SETPIXEL((j >> 16), y1, color);
            j -= decInc;
        }
        return;
    }

    if (longLen > 0) {
        longLen += x1;
        for (int j = 0x8000 + (y1 << 16); x1 <= longLen; ++x1) {
            SETPIXEL(x1, (j >> 16), color);
            j += decInc;
        }
        return;
    }
    longLen += x1;
    for (int j = 0x8000 + (y1 << 16); x1 >= longLen; --x1) {
        SETPIXEL(x1, (j >> 16), color);
        j -= decInc;
    }
}


void CODE_IN_IWRAM DMAFastCopy(void *source, void *dest, u32 count, u32 mode)
{
	REG_DMA3SAD = (u32)source;
	REG_DMA3DAD = (u32)dest;
	REG_DMA3CNT = count | mode;
}

void CODE_IN_IWRAM checkButtons()
{
	buttons[0] = !((*BUTTONS) & BUTTON_A);
	buttons[1] = !((*BUTTONS) & BUTTON_B);
	buttons[2] = !((*BUTTONS) & BUTTON_SELECT);
	buttons[3] = !((*BUTTONS) & BUTTON_START);
	buttons[4] = !((*BUTTONS) & BUTTON_RIGHT);
	buttons[5] = !((*BUTTONS) & BUTTON_LEFT);
	buttons[6] = !((*BUTTONS) & BUTTON_UP);
	buttons[7] = !((*BUTTONS) & BUTTON_DOWN);
	buttons[8] = !((*BUTTONS) & BUTTON_R);
	buttons[9] = !((*BUTTONS) & BUTTON_L);
}


int main(void)
{
	int i, j;
    int x[NUM_POINTS], y[NUM_POINTS];
	int vx[NUM_POINTS], vy[NUM_POINTS];
	unsigned short color;
	int timers;

	vram = VRAM_ADRS;

	// Write int the I/O registers.
#ifdef MODE5
	*REG_DISPCNT = (DISP_MODE5 | DISP_ENABLE_BG2);
	*REG_BG2PA = 170;	// 160 / 240 * 256
	*REG_BG2PB = 0;
	*REG_BG2PC = 0;
	*REG_BG2PD = 204;	// 128 / 160 * 256
	//*REG_BG2X = 120 << FIXED_BITS;
	//*REG_BG2Y = 4 << FIXED_BITS;
#else // MODE3
	*REG_DISPCNT = (DISP_MODE3 | DISP_ENABLE_BG2);
	// Write pixel color into VRAM
	SETPIXEL(115, 80, 0x001F);	// C = 0000000000011111b = R
	SETPIXEL(120, 80, 0x03E0);	// C = 0000011111100000b = G
	SETPIXEL(125, 80, 0x7c00);	// C = 1111100000000000b = B
#endif // MODE5

#if 0
	// Turn on timer0 and set to 256 clocks
	*REG_TM0CNT = TIMER_FREQUENCY_256 | TIMER_ENABLE;
	// Turn on timer1 and grab overflow from timer0
	*REG_TM1CNT = TIMER_OVERFLOW | TIMER_ENABLE;
	// Turn on timer2 and set the system clock
	*REG_TM2CNT = TIMER_FREQUENCY_SYSTEM | TIMER_ENABLE;
	// Turn on timer3 and grab overflow from timer2
	*REG_TM3CNT = TIMER_OVERFLOW | TIMER_ENABLE;
#endif


	xorshiftSeed(777);

    for (i = 0; i < NUM_POINTS; i++) {
        x[i] = xorshift32() % WIDTH;
        y[i] = xorshift32() % HEIGHT;
		vx[i] = xorshift32() % 2;
		if (vx[i] != 1) vx[i] = -1;
		vy[i] = xorshift32() % 2;
		if (vy[i] != 1) vy[i] = -1;
    }

  //buffer = (u8 *)mallocAligned(WIDTH*HEIGHT, 2);
  buffer = (u16 *)mallocAligned(WIDTH*HEIGHT, 2);
	clearColor = 0xffff;
	color = 0xffff;

	while(1) {
		// Clear the draw surface
		DMAFastCopy(clearColor, buffer, WIDTH*HEIGHT, (DMA_SRC_FIXED | DMA_DST_INC | DMA_16 | DMA_ENABLE));
		checkButtons();

		if (buttons[0])
			color = (unsigned short)(xorshift32() & 0xffff);

#if 0
		for (i = 0; i < NUM_POINTS-1; i++) {
        	drawLineEFLA(x[i], y[i], x[i+1], y[i+1], color);
		}
		drawLineEFLA(x[NUM_POINTS-1], y[NUM_POINTS-1], x[0], y[0], color);

    	for (i = 0; i < NUM_POINTS; i++) {
			x[i] += vx[i];
			if ((x[i] >= WIDTH - 2) || (x[i] <= 1)) vx[i] = -vx[i];
			y[i] += vy[i];
			if ((y[i] >= HEIGHT - 2) || (y[i] <= 1)) vy[i] = -vy[i];
		}
#else
		m_world = mRotateX(mesh_rotation.x);
		m_world = mMultiply(mRotateY(mesh_rotation.y), m_world);
		m_world = mMultiply(mRotateZ(mesh_rotation.z), m_world);

		Vector3i p;
		for (i = 0; i < NODECOUNT; i++) {
			p.x = (m_world.m[0][0] * (NODE(i, 0) >> PSHIFT) +
             m_world.m[1][0] * (NODE(i,1) >> PSHIFT) +
             m_world.m[2][0] * (NODE(i,2) >> PSHIFT) +
             m_world.m[3][0]) / PRES;

      p.y = (m_world.m[0][1] * (NODE(i,0) >> PSHIFT) +
             m_world.m[1][1] * (NODE(i,1) >> PSHIFT) +
             m_world.m[2][1] * (NODE(i,2) >> PSHIFT) +
             m_world.m[3][1]) / PRES;

      p.z = (m_world.m[0][2] * (NODE(i,0) >> PSHIFT) +
             m_world.m[1][2] * (NODE(i,1) >> PSHIFT) +
             m_world.m[2][2] * (NODE(i,2) >> PSHIFT) +
             m_world.m[3][2]) / PRES;

      proj_nodes[i][0] = (FOV * p.x) / (FOV + p.z) + HALFW;
      proj_nodes[i][1] = (FOV * p.y) / (FOV + p.z) + HALFH;
    }

    mesh_rotation.x += 10;
    mesh_rotation.y += 8;
    mesh_rotation.z += 7;

		if (mesh_rotation.x > 360) mesh_rotation.x = 0;
    if (mesh_rotation.y > 360) mesh_rotation.y = 0;
    if (mesh_rotation.z > 360) mesh_rotation.z = 0;

    draw_wireframe(proj_nodes, 0xffff);
#endif
		waitForVBlank();
		DMAFastCopy(buffer, vram, WIDTH*HEIGHT, (DMA_IMMEDIATE | DMA_16 | DMA_ENABLE));

		//waitForVSync();
	}

	return 0;
}
