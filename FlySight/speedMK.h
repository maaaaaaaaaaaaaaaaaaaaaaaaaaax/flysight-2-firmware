/*
 * speedMK.h
 *
 *  Created on: Jul 14, 2025
 *      Author: Max
 */

#ifndef SPEEDMK_H_
#define SPEEDMK_H_

#include <stdint.h>
#include "gnss.h"

typedef struct
{
	uint8_t phase;
} FS_speedMK_State_t;

void speedMK_Init();
void speedMK_check_advance_phase(FS_GNSS_Data_t *current);
int32_t speedMK_get_Glideratioerror(FS_GNSS_Data_t *current);

#endif /* SPEEDMK_H_ */
