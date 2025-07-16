/*
 * WScomp.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Max
 */

#include "main.h"
#include "WScomp.h"
#include "math.h"
#include "config.h"
#include "log.h"
#include "audio.h"

#define DEG_TO_RAD M_PI/180.0

void WScomp_calc_P(int32_t lat_dec, int32_t lon_dec);
float WScomp_angle_home();
float WScomp_distance_to_DFP(int32_t lat_dec, int32_t lon_dec);
float WScomp_calc_heading(float, float, float, float);


static FS_WScomp_State_t WScomp_state;

void WScomp_Init() 	//Call at the start of every jump
{
	FS_Log_MKlog("WScomp_Init");

	WScomp_state.phase = 0;

	WScomp_state.starttime = HAL_GetTick();
}

void WScomp_check_advance_phase(FS_GNSS_Data_t *current)
{

	const FS_Config_Data_t *config = FS_Config_Get();


	WScomp_calc_P(current->lat, current->lon);	//Calculate NE from jumpers position to Ground Reference Point

	//FS_Log_MKlog("Px: %i, Py: %i, Pz: %i", (int32_t)navMK_state.P.x, (int32_t)navMK_state.P.y, (int32_t)navMK_state.P.z);

	//int16_t u,v,w,distance_real;
	switch(WScomp_state.phase)
	{
	case 0:	//Wait until minimum exit altitude is reached.
		uint8_t approaching_jumprun = ((current->hMSL - config->dz_elev)/1000 > config->WScomp_exit_min)?1:0;

		if (config->WScomp_Ground_Debug == 1)
		{
			uint32_t elapsedtime = HAL_GetTick() - WScomp_state.starttime;
			if (elapsedtime/1000.0 >= 5)
			{
				approaching_jumprun = 1;
			}
		}

		if (approaching_jumprun)
		{
			FS_Audio_Play("apexalt.wav", config->sp_volume * 5);
			WScomp_state.phase++;
			FS_Log_MKlog("Change to phase %i", WScomp_state.phase);
		}
		break;
	case 1: //Wait until vertical speed is reached to start validation window

		uint8_t jump_detected = 0;

		if (current->velD > (int32_t)config->WScomp_valwindow_speed*1000)
		{
			jump_detected = 1;
		}

		if (config->WScomp_Ground_Debug == 1)
		{
			uint32_t elapsedtime = HAL_GetTick() - WScomp_state.starttime;
			if (elapsedtime/1000.0 >= 10)
			{
				jump_detected = 1;
			}
		}

		if (jump_detected)
		{
			WScomp_state.exittime = HAL_GetTick();
			FS_Audio_Play("exit.wav", config->sp_volume * 5);
			WScomp_state.phase++;
			FS_Log_MKlog("Change to phase %i", WScomp_state.phase);
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
	case 2:	//Wait for delay to start validation window
		uint32_t elapsedtime = HAL_GetTick() - WScomp_state.exittime;
		if (elapsedtime/1000.0 >= config->WScomp_valwindow_delay)
		{
			WScomp_state.Val_Window_Start.Lat = (float)current->lat/10000000;
			WScomp_state.Val_Window_Start.Lon = (float)current->lon/10000000;
			WScomp_state.DFP_heading = WScomp_calc_heading(WScomp_state.Val_Window_Start.Lat, WScomp_state.Val_Window_Start.Lon, (float)config->WScomp_GRP_lat/WSCOMP_WGS84_MULTIPLIER, (float)config->WScomp_GRP_lon/WSCOMP_WGS84_MULTIPLIER)*10;
			WScomp_state.DFP_x = cos(((float)WScomp_state.DFP_heading/10)*DEG_TO_RAD);
			WScomp_state.DFP_y = sin(((float)WScomp_state.DFP_heading/10)*DEG_TO_RAD);
			FS_Log_MKlog("DFP Heading: %i", (int16_t)WScomp_state.DFP_heading);
			WScomp_state.phase++;
			FS_Log_MKlog("Change to phase %i", WScomp_state.phase);
			FS_Audio_Play("alpha.wav", config->sp_volume * 5);
			return;
		}
		break;
	case 3: //wait to get into competition window
		int32_t entered_comp_window = ((current->hMSL - config->dz_elev)/1000 < config->WScomp_compwindow_top)?1:0;


		if (config->WScomp_Ground_Debug == 1)
		{
			uint32_t elapsedtime = HAL_GetTick() - WScomp_state.starttime;
			if (elapsedtime/1000> 180)
				entered_comp_window = 1;
			else
				entered_comp_window = 0;
		}

		if (entered_comp_window == 1)
		{
			WScomp_state.phase++;
			FS_Log_MKlog("Change to phase %i", WScomp_state.phase);
			FS_Audio_Play("bravo.wav", config->sp_volume * 5);
			return;
		}
		break;
	case 4: //wait to exit competition window
		int32_t exited_comp_window = ((current->hMSL - config->dz_elev)/1000 < config->WScomp_compwindow_bottom)?1:0;

		if (config->WScomp_Ground_Debug == 1)
		{
			uint32_t elapsedtime = HAL_GetTick() - WScomp_state.starttime;
			if (elapsedtime/1000> 200)
				exited_comp_window = 1;
			else
				exited_comp_window = 0;
		}

		if (exited_comp_window == 1)
		{
			WScomp_state.phase++;
			FS_Log_MKlog("Change to phase %i", WScomp_state.phase);
			FS_Audio_Play("charlie.wav", config->sp_volume * 5);
			return;
		}
		break;
	case 5: //End
		break;
	default:
		break;
	}
}


int32_t WScomp_get_Headingerror(FS_GNSS_Data_t *current)
{
	int32_t heading_nominal, heading_error;				//[0.1deg]
	int32_t heading_actual = current->heading/10000; 	//[0.1deg]

	if (heading_actual < -1800) heading_actual += 3600;
	if (heading_actual > 1800) heading_actual -= 3600;

	switch (WScomp_state.phase)
	{
	case 2: //Wait for delay to start validation window -> heading_nominal shall be heading to GRP
		heading_nominal = WScomp_angle_home()*10;
		break;
	case 3: //wait to get into competition window -> heading nominal shall be direction of DFP minus perpendicular distance to DFP in meters multiplied by 0.1°.
	case 4: //wait to exit competition window, same as before.
		heading_nominal = WScomp_state.DFP_heading - WScomp_distance_to_DFP(current->lat, current->lon);	//guide jumper back towards designated lane
		break;
	default:
		return INT32_MAX;	//wrong phase for heading
	}

	heading_error = heading_actual - heading_nominal;
	if (heading_error < -1800) heading_error += 3600;
	if (heading_error > 1800) heading_error -= 3600;

	FS_Log_MKlog("Heading actual: %i Heading nominal: %i Heading Error: %i", heading_actual, heading_nominal, heading_error);
	return (heading_error);
}


void WScomp_calc_P(int32_t lat_dec, int32_t lon_dec)	//Calculate NE voordinates from provided position to Ground Reference Point
{
	const FS_Config_Data_t *config = FS_Config_Get();

	float lat = (float)lat_dec/10000000; 	//[°]
	float lon = (float)lon_dec/10000000; 	//[°]

	WScomp_state.P.x = ((float)config->WScomp_GRP_lat/WSCOMP_WGS84_MULTIPLIER - lat)*111000;
	WScomp_state.P.y = ((float)config->WScomp_GRP_lon/WSCOMP_WGS84_MULTIPLIER - lon)*111000*cos(lat*DEG_TO_RAD);
}


float WScomp_angle_home() //Calculate and return heading to Ground Reference Point using P-Matrice
{
	return atan2(WScomp_state.P.y, WScomp_state.P.x)*180/M_PI; //returns heading in degree.
}

float WScomp_distance_to_DFP(int32_t lat_dec, int32_t lon_dec) //positive distance: right side of DFP. negative distance: left side of DFP
{
	const FS_Config_Data_t *config = FS_Config_Get();

	float lat = (float)lat_dec/10000000; 	//[°]
	float lon = (float)lon_dec/10000000; 	//[°]

	float curr_x = (lat - WScomp_state.Val_Window_Start.Lat)*111000;		//vector from start of designated lane to current position of jumper
	float curr_y = (lon - WScomp_state.Val_Window_Start.Lon)*111000*cos(lat*DEG_TO_RAD);

	float t = (curr_x*WScomp_state.DFP_x + curr_y*WScomp_state.DFP_y)/(WScomp_state.DFP_x*WScomp_state.DFP_x + WScomp_state.DFP_y*WScomp_state.DFP_y); //distance from start of designated lane to point on designated lane perpendicular to jumpers position

	float dist_x = curr_x - t*WScomp_state.DFP_x;	//vector from point on designated lane perpendicular to jumpers position
	float dist_y = curr_y - t*WScomp_state.DFP_y;

	float dist =  sqrt(dist_x*dist_x + dist_y*dist_y);	//length of dist-vector

	float crossproduct = WScomp_state.DFP_x*curr_y - WScomp_state.DFP_y*curr_x;
	if (crossproduct < 0)	//Todo: We can probably just use the result of the crossproduct and get rid of the complete dist-vector calculations, since DFP-vector has a length of 1.
	{
		dist = -dist;
	}

	FS_Log_MKlog("T: %i", (int32_t)t);
	FS_Log_MKlog("Dist: %i", (int32_t)dist);
//	FS_Log_MKlog("Crossproduct: %i", (int32_t)crossproduct);

	if (config->WScomp_Ground_Debug == 1)	//For ground debug, guide jumper back to center lane more agressively (20m distance means 20° deviation to designated lane heading)
		dist *= 10;

	//Don't report more than 200m distance, so that a maximum of 20° angle diversion is reached. Todo: Move this to the headingerror-function to be more clean.
	if (dist > 200)
		dist = 200;
	if (dist < (-200))
		dist = -200;

	return dist;

}

float WScomp_calc_heading(float lat1, float lon1, float lat2, float lon2) //Calculates the heading from point 1 to point 2
{
	float x = (lat2 - lat1)*111000;
	float y = (lon2 - lon1)*111000*cos(lat2*DEG_TO_RAD);
	return atan2(y, x)*180/M_PI;
}
