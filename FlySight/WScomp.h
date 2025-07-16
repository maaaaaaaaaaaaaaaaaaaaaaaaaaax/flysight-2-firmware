/*
 * WScomp.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Max
 */

#ifndef WSCOMP_H_
#define WSCOMP_H_

#include <stdint.h>
#include "gnss.h"
#define WSCOMP_WGS84_MULTIPLIER 100000 //change comments in config.h if this value is changed!

typedef struct
{
	float Lat;	//[degree]
	float Lon;	//[degree]
} FS_LatLon_t;

typedef struct
{
	float x;
	float y;
} FS_XYvector_t;

typedef struct
{
	uint8_t phase;
	FS_LatLon_t Val_Window_Start;
	FS_XYvector_t P;
	float DFP_heading; //[0.1deg]
	float DFP_x;
	float DFP_y;
	//uint32_t u;
	//uint32_t v;
	//uint32_t w;
	//FS_Matrice_t M;
	uint32_t starttime;
	uint32_t exittime;
} FS_WScomp_State_t;

void WScomp_Init();
void WScomp_check_advance_phase(FS_GNSS_Data_t *current);
int32_t WScomp_get_Headingerror(FS_GNSS_Data_t *current);


#endif /* WSCOMP_H_ */
