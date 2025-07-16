/*
 * speedMK.c
 *
 *  Created on: Jul 14, 2025
 *      Author: Max
 */

#include "main.h"
#include "speedMK.h"
#include "math.h"
#include "config.h"
#include "log.h"
#include "audio.h"

static FS_speedMK_State_t speedMK_state;

void speedMK_Init()	//Call at beginning of each jump
{
	FS_Log_MKlog("speedMK_Init");
	speedMK_state.phase = 0;
}

void speedMK_check_advance_phase(FS_GNSS_Data_t *current)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	switch(speedMK_state.phase)
	{
	case 0:	//Wait until minimum exit altitude is reached.
		uint8_t approaching_jumprun = ((current->hMSL - config->dz_elev)/1000 > config->WScomp_exit_min)?1:0;

		if (approaching_jumprun)
		{
			FS_Audio_Play("apexalt.wav", config->sp_volume * 5);
			speedMK_state.phase++;
			FS_Log_MKlog("speedMK Change to phase %i", speedMK_state.phase);
		}
		break;
	case 1: //Wait for exit

		uint8_t jump_detected = 0;

		if (current->velD > (int32_t)config->WScomp_valwindow_speed*1000)
		{
			jump_detected = 1;
		}

		if (jump_detected)
		{
			FS_Audio_Play("exit.wav", config->sp_volume * 5);
			speedMK_state.phase++;
			FS_Log_MKlog("speedMK Change to phase %i", speedMK_state.phase);
			return;
		}
		//Check if altitude is too high or too low and give warning signal if so.
		if (FS_Audio_IsIdle())
		{
			if((current->hMSL - config->dz_elev)/1000 > config->WScomp_exit_max)
			{
				FS_Audio_Play("toohigh.wav", config->sp_volume * 5);
			}
			else if((current->hMSL - config->dz_elev)/1000 < config->WScomp_exit_min)
			{
				FS_Audio_Play("toolow.wav", config->sp_volume * 5);
			}
		}
		break;
	case 2: //wait to get into competition window
		int32_t entered_comp_window = ((current->hMSL - config->dz_elev)/1000 < config->WScomp_compwindow_top)?1:0;

		if (entered_comp_window == 1)
		{
			speedMK_state.phase++;
			FS_Log_MKlog("speedMK Change to phase %i", speedMK_state.phase);
			FS_Audio_Play("bravo.wav", config->sp_volume * 5);
			return;
		}
		break;
	case 3:	//Wait to exit competition window
		int32_t exited_comp_window = ((current->hMSL - config->dz_elev)/1000 < config->WScomp_compwindow_bottom)?1:0;

		if (exited_comp_window == 1)
		{
			speedMK_state.phase++;
			FS_Log_MKlog("speedMK Change to phase %i", speedMK_state.phase);
			FS_Audio_Play("charlie.wav", config->sp_volume * 5);
			return;
		}
		break;
	case 4: //End
		break;
	default:
		break;
	}
}

int32_t speedMK_get_Glideratioerror(FS_GNSS_Data_t *current)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	int32_t gliderate_nominal, gliderate_error;				//[0.01]
	int32_t gliderate_actual = (current->gSpeed*100)/(current->velD/10); 				//[0.01]

	switch (speedMK_state.phase)
	{
	case 2: //Wait for delay to start validation window -> heading_nominal shall be heading to GRP
	case 3:
		int32_t current_altitude = (current->hMSL - config->dz_elev)/1000; //[m]
		if (current_altitude > 2800)
		{
			gliderate_nominal = 80;
		}
		else if (current_altitude > 2500)
		{
			gliderate_nominal = 100 - (((current_altitude - 2500)*20)/300); //gradually increase glideratio from 0.8 to 1.0
		}
		else if (current_altitude > 1500)
		{
			gliderate_nominal = 180 - (((current_altitude - 1500)*80)/1000); //gradually increase glideratio from 1.0 to 1.8
		}
		else
		{
			gliderate_nominal = 180;
		}
		break;
	default:
		return INT32_MAX;	//wrong phase, return invalid value so that no sound is produced.
	}

	gliderate_error = gliderate_actual - gliderate_nominal;

	FS_Log_MKlog("Gliderate actual: %i Gliderate nominal: %i Gliderate Error: %i", gliderate_actual, gliderate_nominal, gliderate_error);
	return (gliderate_error);
}
