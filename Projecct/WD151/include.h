#ifndef _include_H_
#define _include_H_



#include "headfile.h"
#include "mymath.h"
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

//#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)  /*!< JTAG-DP Disabled and SW-DP Enabled */

#define HW_TYPE	1
#define HW_VER	3
#define BL_VER	100
#define PT_VER	400

typedef struct
{
	float x;
	float y;
	float z;
} _xyz_f_st;

typedef struct 
{
  float x;
	float y;
	float z;
}xyz_f_t;

typedef struct 
{
  short int x;
	short int y;
	short int z;

}xyz_s16_t;


typedef struct
{
	short int x;
	short int y;
	short int z;
} _xyz_s16_st;

typedef struct
{
	float x;
	float y;
	float z;
}
__attribute__((packed)) _xyz_f_st_pk;

typedef struct
{
	short int x;
	short int y;
	short int z;
}__attribute__((packed)) _xyz_s16_st_pk;


enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_Y ,
 G_X ,
 G_Z ,
 TEM ,
 ITEMS ,
};

#endif


