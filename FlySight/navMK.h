/*
 * navMK.h
 *
 *  Created on: May 4, 2025
 *      Author: Max
 */

#ifndef NAVMK_H_
#define NAVMK_H_

#include <stdint.h>
#include "gnss.h"
#define NAVMK_WGS84_MULTIPLIER 100000 //change comments in config.h if this value is changed!


typedef struct
{
	float x;
	float y;
	float z;
} FS_XYZvector_t;

typedef struct
{
	FS_XYZvector_t x;
	FS_XYZvector_t y;
	FS_XYZvector_t z;
} FS_Matrice_t;

typedef struct
{
	uint8_t phase;
	FS_XYZvector_t P;
	uint32_t u;
	uint32_t v;
	uint32_t w;
	FS_Matrice_t M;
	uint32_t starttime;
} FS_navMK_State_t;


void navMK_Init();
void navMK_check_advance_phase(FS_GNSS_Data_t *current);
int32_t navMK_get_Headingerror(FS_GNSS_Data_t *current);

#endif /* NAVMK_H_ */
