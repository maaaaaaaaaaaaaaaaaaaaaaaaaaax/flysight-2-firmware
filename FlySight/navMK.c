/*
 * navMK.c
 *
 *  Created on: May 4, 2025
 *      Author: Max
 */
#include "main.h"
#include "navMK.h"
#include "math.h"
#include "config.h"
#include "log.h"
#include "audio.h"

static FS_navMK_State_t navMK_state;


#define DEG_TO_RAD M_PI/180.0

void navMK_calc_P(int32_t lat_dec, int32_t lon_dec, int32_t alt_dec);
uint16_t abs_diff_angle(int16_t angle1, int16_t angle2);
float navMK_angle_home();


void navMK_Init()	//Init, should be called before each jump.
{
	FS_Log_MKlog("navMK_Init");
	const FS_Config_Data_t *config = FS_Config_Get();

	navMK_state.phase = 0;

	float phi = atan(1.0/((double)config->av_glide_ratio/100.0)); //phi in rad
	FS_Log_MKlog("phi*1000: %i", (int32_t)(phi*1000.0));
	FS_XYZvector_t L1, L2, L3;	// [m] NED vector for each leg

	L1.x = cos((float)config->leg1_heading*DEG_TO_RAD)*cos(phi);	//using NED -> xyz
	L1.y = sin((float)config->leg1_heading*DEG_TO_RAD)*cos(phi);
	L1.z = sin(phi);

	L2.x = cos((float)config->leg2_heading*DEG_TO_RAD)*cos(phi);
	L2.y = sin((float)config->leg2_heading*DEG_TO_RAD)*cos(phi);
	L2.z = sin(phi);

	L3.x = cos((float)config->leg3_heading*DEG_TO_RAD)*cos(phi);
	L3.y = sin((float)config->leg3_heading*DEG_TO_RAD)*cos(phi);
	L3.z = sin(phi);

	FS_Log_MKlog("L1x: %i, L1y: %i, L1z: %i, ", (int32_t)(L1.x*1000.0), (int32_t)(L1.y*1000.0), (int32_t)(L1.z*1000.0) );
	FS_Log_MKlog("L2x: %i, L2y: %i, L2z: %i, ", (int32_t)(L2.x*1000.0), (int32_t)(L2.y*1000.0), (int32_t)(L2.z*1000.0) );
	FS_Log_MKlog("L3x: %i, L3y: %i, L3z: %i, ", (int32_t)(L3.x*1000.0), (int32_t)(L3.y*1000.0), (int32_t)(L3.z*1000.0) );
	float det = L1.x*L2.y*L3.z + L2.x*L3.y*L1.z + L3.x*L1.y*L2.z - L1.z*L2.y*L3.x - L2.z*L3.y*L1.x - L3.z*L1.y*L2.x; //determinant of vector

	FS_Log_MKlog("det*1000: %i", (int32_t)(det*1000.0));

	navMK_state.M.x.x = (L2.y*L3.z-L2.z*L3.y)/det;	//Solving Matrice used to solve length of each vector
	navMK_state.M.x.y = (L2.z*L3.x-L2.x*L3.z)/det;
	navMK_state.M.x.z = (L2.x*L3.y-L2.y*L3.x)/det;

	navMK_state.M.y.x = (L1.z*L3.y-L1.y*L3.z)/det;
	navMK_state.M.y.y = (L1.x*L3.z-L1.z*L3.x)/det;
	navMK_state.M.y.z = (L1.y*L3.x-L1.x*L3.y)/det;

	navMK_state.M.z.x = (L1.y*L2.z-L1.z*L2.y)/det;
	navMK_state.M.z.y = (L1.z*L2.x-L1.x*L2.z)/det;
	navMK_state.M.z.z =	(L1.x*L2.y-L1.y*L2.x)/det;

	FS_Log_MKlog("Mxx: %i, Mxy: %i, Mxz: %i, ", (int32_t)(navMK_state.M.x.x*1000.0), (int32_t)(navMK_state.M.x.y*1000.0), (int32_t)(navMK_state.M.x.z*1000.0) );
	FS_Log_MKlog("Myx: %i, Myy: %i, Myz: %i, ", (int32_t)(navMK_state.M.y.x*1000.0), (int32_t)(navMK_state.M.y.y*1000.0), (int32_t)(navMK_state.M.y.z*1000.0) );
	FS_Log_MKlog("Mzx: %i, Mzy: %i, Mzz: %i, ", (int32_t)(navMK_state.M.z.x*1000.0), (int32_t)(navMK_state.M.z.y*1000.0), (int32_t)(navMK_state.M.z.z*1000.0) );

	navMK_state.starttime = HAL_GetTick();
}

void navMK_check_advance_phase(FS_GNSS_Data_t *current)
{

	const FS_Config_Data_t *config = FS_Config_Get();

	static uint8_t reachable;

	navMK_calc_P(current->lat, current->lon, current->hMSL - config->dz_elev);	//Calculate the NED values from jumpers position to pull point

	FS_Log_MKlog("Px: %i, Py: %i, Pz: %i", (int32_t)navMK_state.P.x, (int32_t)navMK_state.P.y, (int32_t)navMK_state.P.z);

	int16_t u,v,w,distance_real;
	switch(navMK_state.phase)
	{
	case 0: //Wait for jumper to approach exit altitude (go to next phase 500m below exit altitude)
		uint8_t approaching_jumprun = ((current->hMSL - config->dz_elev)/1000 > config->exit_altitude - 500)?1:0;

		if (config->navMK_Ground_Debug == 1) approaching_jumprun = 1;

		if (approaching_jumprun)
		{
			FS_Audio_Play("apexalt.wav", config->sp_volume * 5);
			navMK_state.phase++;
			FS_Log_MKlog("Change to phase %i", navMK_state.phase);
			reachable = 0;
		}
		break;
	case 1: //Wait for exit (vertical velocity exceeding threshold). Tell jumper is pull point is reachable (and also if it is out of reach again)
		u = navMK_state.M.x.x * navMK_state.P.x + navMK_state.M.x.y * navMK_state.P.y + navMK_state.M.x.z * navMK_state.P.z; //Calculate all 3 leg length
		v = navMK_state.M.y.x * navMK_state.P.x + navMK_state.M.y.y * navMK_state.P.y + navMK_state.M.y.z * navMK_state.P.z;
		w = navMK_state.M.z.x * navMK_state.P.x + navMK_state.M.z.y * navMK_state.P.y + navMK_state.M.z.z * navMK_state.P.z;
		FS_Log_MKlog("u: %i, v: %i, w: %i", u, v, w);
		if (u < 0 || v < 0 || w < 0)
		{//pull point not reachable, at least one vector is negative.
			if (reachable == 1)
			{//pull point was reachable in last check -> inform jumper that it is not reachable anymore.
				reachable = 0;
				FS_Log_MKlog("Pull Point out of reach");
				FS_Audio_Play("ppoof.wav", config->sp_volume * 5);
			}
		}
		else
		{//pull point reachable, all vectors positive.
			if (reachable == 0)
			{//pull point was not reachable in last check -> inform jumper that it is now reachable.
				reachable = 1;
				FS_Log_MKlog("Pull Point reachable");
				FS_Audio_Play("ppreach.wav", config->sp_volume * 5);
			}
		}

		uint8_t jump_detected = 0;

		if (current->velD > (int32_t)config->navMK_jump_threshold*278) //1km/h = 1000000mm/h = 278mm/sec
		{
			jump_detected = 1;
		}

		if (config->navMK_Ground_Debug == 1)
		{
			uint32_t elapsedtime = HAL_GetTick() - navMK_state.starttime;
			if (elapsedtime/1000.0 >= 10)
			{
				jump_detected = 1;
			}
		}

		if (jump_detected)
		{
			FS_Audio_Play("exit.wav", config->sp_volume * 5);
			navMK_state.phase += 4 - config->navMK_numLegs; //3 legs -> go to next phase. 2 leg-> skip next phase. 1 leg-> skip next 2 phases
			FS_Log_MKlog("Change to phase %i", navMK_state.phase);
		}

		break;
	case 2:	//Leg1, solve for all 3 vectors and switch to next leg if length of leg1 is smaller than the threshold distance.
		u = navMK_state.M.x.x * navMK_state.P.x + navMK_state.M.x.y * navMK_state.P.y + navMK_state.M.x.z * navMK_state.P.z;
		v = navMK_state.M.y.x * navMK_state.P.x + navMK_state.M.y.y * navMK_state.P.y + navMK_state.M.y.z * navMK_state.P.z;
		w = navMK_state.M.z.x * navMK_state.P.x + navMK_state.M.z.y * navMK_state.P.y + navMK_state.M.z.z * navMK_state.P.z;
		FS_Log_MKlog("u: %i, v: %i, w: %i", u, v, w);
		if (u < 0 || v < 0 || w < 0 || u < config->navMK_preturn_alert*abs_diff_angle(config->leg1_heading, config->leg2_heading) || (u + v) < config->navMK_preturn_alert*abs_diff_angle(config->leg1_heading, config->leg3_heading))
		{
			navMK_state.phase++;
			FS_Log_MKlog("Change to phase %i", navMK_state.phase);
			FS_Audio_Play("alpha.wav", config->sp_volume * 5);
			return;
		}
		break;
	case 3: //Leg2, get the line of sight distance to the pull point and compare it to the reachable distance (calculated via the glide ratio). Advance to leg3 once line of sight distance is smaller than reachable distance (not completely true, since we use the preturn alert)
		distance_real = sqrt(navMK_state.P.x*navMK_state.P.x+navMK_state.P.y*navMK_state.P.y);
		int32_t distance_reachable = ((float)config->av_glide_ratio/100.0)*(current->hMSL - config->dz_elev - config->pull_alt*1000)/1000;
		if (config->navMK_Ground_Debug == 1)
		{
			uint32_t elapsedtime = HAL_GetTick() - navMK_state.starttime;;
			distance_reachable = ((float)config->av_glide_ratio/100.0)*(120 - elapsedtime/1000.0);
		}
		FS_Log_MKlog("Distance real: %i, Distance reachable: %i", distance_real, distance_reachable);
		if ((distance_reachable - config->navMK_preturn_alert*abs_diff_angle(config->leg2_heading, navMK_angle_home())) < distance_real) //Advance to phase 2 once distance to Pull Point is higher than distance reachable via flight glide ratio * height difference
		{
			navMK_state.phase++;
			FS_Log_MKlog("Change to phase %i", navMK_state.phase);
			FS_Audio_Play("bravo.wav", config->sp_volume * 5);
			return;
		}
		break;
	case 4: //Leg3, just guide jumper towards pull point. move towards next point once altitude is below pull altitude.
		distance_real = sqrt(navMK_state.P.x*navMK_state.P.x+navMK_state.P.y*navMK_state.P.y);
		int32_t current_altitude = (current->hMSL - config->dz_elev)/1000;
		int32_t pull_altitude = config->pull_alt;
		if (config->navMK_Ground_Debug == 1)
		{
			current_altitude = 120- (HAL_GetTick() - navMK_state.starttime)/1000;
			pull_altitude = 0;
		}
		FS_Log_MKlog("Distance real: %i, current alt: %i, pull alt: %i", distance_real, current_altitude, pull_altitude);
		if (current_altitude < pull_altitude)
		{
			navMK_state.phase++;
			FS_Log_MKlog("Change to phase %i", navMK_state.phase);
			FS_Audio_Play("charlie.wav", config->sp_volume * 5);
			return;
		}
		break;
	case 5: //End, do nothing.
		break;
	default:
		break;
	}
}

void navMK_calc_P(int32_t lat_dec, int32_t lon_dec, int32_t alt_dec) //Calculate NED coordinates from provided position to pull point.
{
	const FS_Config_Data_t *config = FS_Config_Get();


	float lat = (float)lat_dec/10000000; 	//[°]
	float lon = (float)lon_dec/10000000; 	//[°]
	float alt = (float)alt_dec/1000 ; 		//[m]

	navMK_state.P.x = ((float)config->pull_lat/NAVMK_WGS84_MULTIPLIER - lat)*111000;
	navMK_state.P.y = ((float)config->pull_lon/NAVMK_WGS84_MULTIPLIER - lon)*111000*cos(lat*DEG_TO_RAD);
	navMK_state.P.z = alt - (float)config->pull_alt;

	if (config->navMK_Ground_Debug == 1)
	{
		uint32_t elapsedtime = HAL_GetTick() - navMK_state.starttime;
		navMK_state.P.z = 120 - elapsedtime/1000.0;
	}
}

int32_t navMK_get_Headingerror(FS_GNSS_Data_t *current)	//calculate and return heading error based on leg headings and current phase.
{
	const FS_Config_Data_t *config = FS_Config_Get();

	int32_t heading_nominal, heading_error;				//[0.1deg]
	int32_t heading_actual = current->heading/10000; 	//[0.1deg]

	switch (navMK_state.phase)
	{
	case 2:
		heading_nominal = config->leg1_heading*10;
		break;
	case 3:
		heading_nominal = config->leg2_heading*10;
		break;
	case 4:
		heading_nominal = navMK_angle_home()*10;
		break;
	default:
		return INT32_MAX;	//wrong phase for heading, provide "invalid value" so that no sound is produced.
	}

	heading_error = heading_actual - heading_nominal;
	if (heading_error < -1800) heading_error += 3600;
	if (heading_error > 1800) heading_error -= 3600;

	FS_Log_MKlog("Heading actual: %i Heading nominal: %i Heading Error: %i", heading_actual, heading_nominal, heading_error);
	return (heading_error);
}

uint16_t abs_diff_angle(int16_t angle1, int16_t angle2)	//Calculate and return absolute angle difference between two angles
{
	if (angle1<0) angle1 +=360;
	if (angle2<0) angle2 +=360;

	int16_t diff = angle2-angle1;
	if (diff > 180) diff -= 360;
	if (diff < -180) diff += 360;

	diff = diff<0 ? -1*diff : diff;	//return absolut value;
	return diff;
}

float navMK_angle_home()	//Calculate angle to get home based on NED values saved in P-Matrice.
{
	return atan2(navMK_state.P.y, navMK_state.P.x)*180/M_PI;
}
