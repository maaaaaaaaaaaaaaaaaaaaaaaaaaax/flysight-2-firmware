/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

#define FS_CONFIG_MAX_ALARMS    20
#define FS_CONFIG_MAX_WINDOWS   2
#define FS_CONFIG_MAX_SPEECH    3

#define FS_CONFIG_MODEL_PORTABLE     0
#define FS_CONFIG_MODEL_STATIONARY   2
#define FS_CONFIG_MODEL_PEDESTRIAN   3
#define FS_CONFIG_MODEL_AUTOMOTIVE   4
#define FS_CONFIG_MODEL_SEA          5
#define FS_CONFIG_MODEL_AIRBORNE_1G  6
#define FS_CONFIG_MODEL_AIRBORNE_2G  7
#define FS_CONFIG_MODEL_AIRBORNE_4G  8

#define FS_CONFIG_MODE_HORIZONTAL_SPEED          0
#define FS_CONFIG_MODE_VERTICAL_SPEED            1
#define FS_CONFIG_MODE_GLIDE_RATIO               2
#define FS_CONFIG_MODE_INVERSE_GLIDE_RATIO       3
#define FS_CONFIG_MODE_TOTAL_SPEED               4
#define FS_CONFIG_MODE_DIRECTION_TO_DESTINATION  5
#define FS_CONFIG_MODE_DISTANCE_TO_DESTINATION   6
#define FS_CONFIG_MODE_DIRECTION_TO_BEARING      7
#define FS_CONFIG_MODE_MAGNITUDE_OF_VALUE_1      8
#define FS_CONFIG_MODE_CHANGE_IN_VALUE_1         9
#define FS_CONFIG_MODE_LEFT_RIGHT                10
#define FS_CONFIG_MODE_DIVE_ANGLE                11
#define FS_CONFIG_MODE_ALTITUDE                  12
#define FS_CONFIG_MODE_MK		 				 13	//Debug Mode to test new functionalities
#define FS_CONFIG_MODE_NAVMK					 14	//Mode to navigate through 3 different legs (headings) to end up at specific pull point (lat lon alt)
#define FS_CONFIG_MODE_WSCOMP					 15	//Mode to navigate yourself to the center line of the designated lane according to FAI rules
#define FS_CONFIG_MODE_MKSPEED					 16 //Mode to indicate the perfect glide ratio for a wingsuit performance speed run

#define FS_CONFIG_UNITS_KMH     0
#define FS_CONFIG_UNITS_MPH     1
#define FS_CONFIG_UNITS_KNOTS   2

#define FS_CONFIG_UNITS_METERS  0
#define FS_CONFIG_UNITS_FEET    1
#define FS_CONFIG_UNITS_NM      2

#define FS_CONFIG_RATE_ONE_HZ   650
#define FS_CONFIG_RATE_FLATLINE UINT16_MAX

typedef struct
{
	int32_t elev;
	uint8_t type;
	char    filename[9];
} FS_Config_Alarm_t;

typedef struct
{
	int32_t top;
	int32_t bottom;
} FS_Config_Window_t;

typedef struct
{
	uint8_t mode;
	uint8_t units;
	int32_t decimals;
} FS_Config_Speech_t;

typedef struct
{
	uint8_t  model;
	uint16_t rate;

	uint8_t  mode;
	int32_t  min;
	int32_t  max;
	uint8_t  limits;
	uint16_t volume;

	uint8_t  mode_2;
	int32_t  min_2;
	int32_t  max_2;
	int32_t  min_rate;
	int32_t  max_rate;
	uint8_t  flatline;

	uint8_t  chirp_control;					//MK: Set to 1 to enable chirp control. Chirp always starts in the exact middle of start freq and end freq and goes up or down from here.
	int32_t  chirp_control_min_freq;		//[Hz] Lowest chirp start frequency
	int32_t  chirp_control_max_freq;		//[Hz] Highest chirp end frequency
	int16_t  chirp_control_rate; 			//[0.01Hz] Sets rate of chirps. Value of 100 equals 1Hz.
	int32_t  chirp_control_min_duration; 	//[ms] Minimum duration of a chirp
	int32_t  chirp_control_max_duration; 	//[ms] Maximum duration of a chirp. Make sure not to go higher than allowed by the rate of chirps. (If rate of chirps is at 1Hz, max duration must be below 1000ms to make sense)

	//################## MK: whole next block added for navMK
	int32_t pull_lat;	 	//[0.00001°] verify with NAVMK_WGS84_MULTIPLIER (gives us a resolution of 1.11m)
	int32_t pull_lon;		//[0.00001°] verify with NAVMK_WGS84_MULTIPLIER (gives us a resolution of 1.11m at equator and less anywhere else)
	int32_t pull_alt;		//[m]
	int16_t leg1_heading; 	//[°]
	int16_t leg2_heading; 	//[°]
	int16_t leg3_heading; 	//[°]
	uint8_t	navMK_numLegs;	//[1]
	uint16_t av_glide_ratio; //[0.01]
	uint16_t exit_altitude;	//[m]
	uint8_t navMK_jump_threshold; //[km/h]
	uint8_t navMK_preturn_alert; //[m/deg] how many meters before a turn to switch to the next leg. (Prealert distance = preturn_alert * turnangle)
	uint8_t navMK_Ground_Debug; //used to debug on ground using time instead of altitude

	//################## MK: whole next block added for WS Performance competition
	uint16_t WScomp_compwindow_top;		//[m] Top of the competition window (2500m for FAI)
	uint16_t WScomp_compwindow_bottom;	//[m] Bottom of the competition window (1500m for FAI)
	uint16_t WScomp_exit_max;			//[m] Maximum exit altitude for jump to count (3353m for FAI)
	uint16_t WScomp_exit_min;			//[m] Minimum exit altitude for jump to count (3200m for FAI)
	uint8_t WScomp_valwindow_speed;		//[m/s] Vertical speed to be exceeded to start validation window (10m/s for FAI)
	uint8_t WScomp_valwindow_delay;		//[s] Delay between Exceeding vertical speed and actual start of validation window (9s for FAI).
	int32_t WScomp_GRP_lat;				//[0.00001°] verify with WSCOMP_WGS84_MULTIPLIER (gives us a resolution of 1.11m) Ground Reference point Latitude. Used in combination with position at start of validation window to calculate designated flight path.
	int32_t WScomp_GRP_lon;				//[0.00001°] verify with WSCOMP_WGS84_MULTIPLIER (gives us a resolution of 1.11m at equator and less anywhere else)Ground Reference point Longitude. Used in combination with position at start of validation window to calculate designated flight path.
	uint16_t WScomp_DFP_width;			//[m] Total width of designated flight path (600m for FAI)
	uint8_t WScomp_Ground_Debug; 		//used to debug on ground using time instead of altitude


	uint16_t sp_rate;
	uint16_t sp_volume;

	FS_Config_Speech_t speech[FS_CONFIG_MAX_SPEECH];
	uint8_t  num_speech;

	int32_t  threshold;
	int32_t  hThreshold;

	uint8_t  use_sas;
	int32_t  tz_offset;

	uint8_t  init_mode;
	char     init_filename[9];

	int32_t  alarm_window_above;
	int32_t  alarm_window_below;
	int32_t  dz_elev; 	//[in mm] Inside config file it's saved as m, but when the config file is read, the value is converted from m to mm.

	FS_Config_Alarm_t alarms[FS_CONFIG_MAX_ALARMS];
	uint8_t  num_alarms;

	uint8_t  alt_units;
	int32_t  alt_step;

	FS_Config_Window_t windows[FS_CONFIG_MAX_WINDOWS];
	uint8_t  num_windows;

	uint8_t  enable_audio;
	uint8_t  enable_logging;
	uint8_t  enable_vbat;
	uint8_t  enable_mic;
	uint8_t  enable_imu;
	uint8_t  enable_gnss;
	uint8_t  enable_baro;
	uint8_t  enable_hum;
	uint8_t  enable_mag;
	uint8_t  ble_tx_power;
	uint8_t  enable_raw;
	uint8_t  enable_mklog; //MK: Added for debug logging
	uint8_t  cold_start;

	uint8_t  baro_odr;
	uint8_t  hum_odr;
	uint8_t  mag_odr;
	uint8_t  accel_odr;
	uint8_t  accel_fs;
	uint8_t  gyro_odr;
	uint8_t  gyro_fs;

	int32_t  lat;
	int32_t  lon;
	int16_t  bearing;
	uint16_t end_nav;
	uint16_t max_dist;
	uint16_t min_angle;
} FS_Config_Data_t;

typedef enum {
	FS_CONFIG_OK = 0,
	FS_CONFIG_ERR
} FS_Config_Result_t;

void FS_Config_Init(void);
FS_Config_Result_t FS_Config_Read(const char *filename);
FS_Config_Result_t FS_Config_Write(const char *filename);
const FS_Config_Data_t *FS_Config_Get(void);

#endif /* CONFIG_H_ */
