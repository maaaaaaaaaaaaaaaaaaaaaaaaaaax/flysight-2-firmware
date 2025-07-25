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

#include "main.h"
#include "app_common.h"
#include "config.h"
#include "ff.h"
#include "navMK.h"
#include "WScomp.h"

#define CONFIG_FIRST_ALARM  0x01
#define CONFIG_FIRST_WINDOW 0x02
#define CONFIG_FIRST_SPEECH 0x04

static FS_Config_Data_t config;
static FIL configFile;

static const char defaultConfig[] =
		"; FlySight - http://flysight.ca\n"
		"\n"
		"; GPS settings\n"
		"\n"
		"Model:     7     ; Dynamic model\n"
		"                 ;   0 = Portable\n"
		"                 ;   2 = Stationary\n"
		"                 ;   3 = Pedestrian\n"
		"                 ;   4 = Automotive\n"
		"                 ;   5 = Sea\n"
		"                 ;   6 = Airborne with < 1 G acceleration\n"
		"                 ;   7 = Airborne with < 2 G acceleration\n"
		"                 ;   8 = Airborne with < 4 G acceleration\n"
		"Rate:      200   ; Measurement rate (ms)\n"
		"\n"
		"; Tone settings\n"
		"\n"
		"Mode:      2     ; Measurement mode\n"
		"                 ;   0 = Horizontal speed\n"
		"                 ;   1 = Vertical speed\n"
		"                 ;   2 = Glide ratio\n"
		"                 ;   3 = Inverse glide ratio\n"
		"                 ;   4 = Total speed\n"
		"                 ;   11 = Dive angle\n"
		"Min:       0     ; Lowest pitch value\n"
		"                 ;   cm/s        in Mode 0, 1, or 4\n"
		"                 ;   ratio * 100 in Mode 2 or 3\n"
		"                 ;   degrees     in Mode 11\n"
		"Max:       300   ; Highest pitch value\n"
		"                 ;   cm/s        in Mode 0, 1, or 4\n"
		"                 ;   ratio * 100 in Mode 2 or 3\n"
		"                 ;   degrees     in Mode 11\n"
		"Limits:    1     ; Behaviour when outside bounds\n"
		"                 ;   0 = No tone\n"
		"                 ;   1 = Min/max tone\n"
		"                 ;   2 = Chirp up/down\n"
		"                 ;   3 = Chirp down/up\n"
		"Volume:    6     ; 0 (min) to 8 (max)\n"
		"\n"
		"; Rate settings\n"
		"\n"
		"Mode_2:    9     ; Determines tone rate\n"
		"                 ;   0 = Horizontal speed\n"
		"                 ;   1 = Vertical speed\n"
		"                 ;   2 = Glide ratio\n"
		"                 ;   3 = Inverse glide ratio\n"
		"                 ;   4 = Total speed\n"
		"                 ;   8 = Magnitude of Value 1\n"
		"                 ;   9 = Change in Value 1\n"
		"                 ;   11 = Dive angle\n"
		"Min_Val_2: 300   ; Lowest rate value\n"
		"                 ;   cm/s          when Mode 2 = 0, 1, or 4\n"
		"                 ;   ratio * 100   when Mode 2 = 2 or 3\n"
		"                 ;   percent * 100 when Mode 2 = 9\n"
		"                 ;   degrees       when Mode 2 = 11\n"
		"Max_Val_2: 1500  ; Highest rate value\n"
		"                 ;   cm/s          when Mode 2 = 0, 1, or 4\n"
		"                 ;   ratio * 100   when Mode 2 = 2 or 3\n"
		"                 ;   percent * 100 when Mode 2 = 9\n"
		"                 ;   degrees       when Mode 2 = 11\n"
		"Min_Rate:  100   ; Minimum rate (Hz * 100)\n"
		"Max_Rate:  500   ; Maximum rate (Hz * 100)\n"
		"Flatline:  0     ; Flatline at minimum rate\n"
		"                 ;   0 = No\n"
		"                 ;   1 = Yes\n"
		"\n"
		"; Speech settings\n"
		"\n"
		"Sp_Rate:   0     ; Speech rate (s)\n"
		"                 ;   0 = No speech\n"
		"Sp_Volume: 6     ; 0 (min) to 8 (max)\n"
		"\n"
		"Sp_Mode:   2     ; Speech mode\n"
		"                 ;   0 = Horizontal speed\n"
		"                 ;   1 = Vertical speed\n"
		"                 ;   2 = Glide ratio\n"
		"                 ;   3 = Inverse glide ratio\n"
		"                 ;   4 = Total speed\n"
		"                 ;   11 = Dive angle\n"
		"                 ;   12 = Altitude above DZ_Elev\n"
		"Sp_Units:  1     ; Speech units\n"
		"                 ;   0 = km/h or m\n"
		"                 ;   1 = mph or feet\n"
		"Sp_Dec:    1     ; Speech precision\n"
		"                 ;   Altitude step in Mode 12\n"
		"                 ;   Decimal places in all other Modes\n"
		"\n"
		"; Thresholds\n"
		"\n"
		"V_Thresh:  1000  ; Minimum vertical speed for tone (cm/s)\n"
		"H_Thresh:  0     ; Minimum horizontal speed for tone (cm/s)\n"
		"\n"
		"; Miscellaneous\n"
		"\n"
		"Use_SAS:   1     ; Use skydiver's airspeed\n"
		"                 ;   0 = No\n"
		"                 ;   1 = Yes\n"
		"TZ_Offset: 0     ; Timezone offset of output files in seconds\n"
		"                 ;   -14400 = UTC-4 (EDT)\n"
		"                 ;   -18000 = UTC-5 (EST, CDT)\n"
		"                 ;   -21600 = UTC-6 (CST, MDT)\n"
		"                 ;   -25200 = UTC-7 (MST, PDT)\n"
		"                 ;   -28800 = UTC-8 (PST)\n"
		"\n"
		"; Initialization\n"
		"\n"
		"Init_Mode: 0     ; When the FlySight is powered on\n"
		"                 ;   0 = Do nothing\n"
		"                 ;   1 = Test speech mode\n"
		"                 ;   2 = Play file\n"
		"Init_File: 0     ; File to be played\n"
		"\n"
		"; Alarm settings\n"
		"\n"
		"; WARNING: GPS measurements depend on very weak signals\n"
		";          received from orbiting satellites. As such, they\n"
		";          are prone to interference, and should NEVER be\n"
		";          relied upon for life saving purposes.\n"
		"\n"
		";          UNDER NO CIRCUMSTANCES SHOULD THESE ALARMS BE\n"
		";          USED TO INDICATE DEPLOYMENT OR BREAKOFF ALTITUDE.\n"
		"\n"
		"; NOTE:    Alarm elevations are given in meters above ground\n"
		";          elevation, which is specified in DZ_Elev.\n"
		"\n"
		"Win_Above:     0 ; Window above each alarm (m)\n"
		"Win_Below:     0 ; Window below each alarm (m)\n"
		"DZ_Elev:       0 ; Ground elevation (m above sea level)\n"
		"\n"
		"Alarm_Elev:    0 ; Alarm elevation (m above ground level)\n"
		"Alarm_Type:    0 ; Alarm type\n"
		"                 ;   0 = No alarm\n"
		"                 ;   1 = Beep\n"
		"                 ;   2 = Chirp up\n"
		"                 ;   3 = Chirp down\n"
		"                 ;   4 = Play file\n"
		"Alarm_File:    0 ; File to be played\n"
		"\n"
		"; Altitude mode settings\n"
		"\n"
		"; WARNING: GPS measurements depend on very weak signals\n"
		";          received from orbiting satellites. As such, they\n"
		";          are prone to interference, and should NEVER be\n"
		";          relied upon for life saving purposes.\n"
		"\n"
		";          UNDER NO CIRCUMSTANCES SHOULD ALTITUDE MODE BE\n"
		";          USED TO INDICATE DEPLOYMENT OR BREAKOFF ALTITUDE.\n"
		"\n"
		"; NOTE:    Altitude is given relative to ground elevation,\n"
		";          which is specified in DZ_Elev. Altitude mode will\n"
		";          not function below 1500 m above ground.\n"
		"\n"
		"Alt_Units:     1 ; Altitude units\n"
		"                 ;   0 = m\n"
		"                 ;   1 = ft\n"
		"Alt_Step:      0 ; Altitude between announcements\n"
		"                 ;   0 = No altitude\n"
		"\n"
		"; Silence windows\n"
		"\n"
		"; NOTE:    Silence windows are given in meters above ground\n"
		";          elevation, which is specified in DZ_Elev. Tones\n"
		";          will be silenced during these windows and only\n"
		";          alarms will be audible.\n"
		"\n"
		"Win_Top:       0 ; Silence window top (m)\n"
		"Win_Bottom:    0 ; Silence window bottom (m)\n"
		"\n"
		"; MK Nav Settings\n"
		"navMK_Pull_Lat:                 4445081  ; [0.00001°] Latitude of designated pull point\n"
		"navMK_Pull_Lon:                 604054   ; [0.00001°] Longitude of designated pull point\n"
		"navMK_Pull_Alt:                 1500     ; [m] Altitude of designated pull point\n"
		"navMK_Leg1:                     135      ; [degree]\n"
		"navMK_Leg2:                     90       ; [degree]\n"
		"navMK_Leg3:                     0        ; [degree]\n"
		"navMK_numLegs:                  3        ; [1] Amount of legs(if only 2 legs are used, leg1 is ignored. Fill out leg2 and leg3. If only one leg is used, use leg3."
		"navMK_Average_Glide_Ratio:      100      ; [0.01]\n"
		"navMK_Ground_Debug:             0        ; Set to 1 to use time instead of altitude (for debugging on the ground)"
		"\n"
		"WScomp_compwindow_top:          2600; [m] Top of the competition window, bravo-sound will play once when crossing (2500m for FAI)\n"
		"WScomp_compwindow_bottom:       1500; [m] Bottom of the competition window, charlie-sound will play once when crossing (1500m for FAI)\n"
		"WScomp_exit_max:                3350; [m] Maximum exit altitude for jump to count, too-high-sound will play if higher (3353m for FAI)\n"
		"WScomp_exit_min:                3200; [m] Minimum exit altitude for jump to count, too-low-sound will play if lower (starting once this boundary has crossed for the first time) (3200m for FAI)\n"
		"WScomp_valwindow_speed:         10; [m/s] Vertical speed to be exceeded to start validation window, exit-sound will play once when speed is exceeded (10m/s for FAI)\n"
		"WScomp_valwindow_delay:         9; [s] Delay between Exceeding vertical speed and actual start of validation window, alpha-sound will play once after this delay (9s for FAI).\n"
		"WScomp_GRP_lat:                 5804930; [0.00001°] Ground Reference point Latitude. Used in combination with position at start of validation window to calculate designated flight path.\n"
		"WScomp_GRP_lon:                 1279159; [0.00001°] Ground Reference point Longitude. Used in combination with position at start of validation window to calculate designated flight path.\n"
		"WScomp_DFP_width:               600; [m] Total width of designated flight path, currently not used for anything (600m for FAI)\n"
		"WScomp_Ground_Debug:            0; Set to 1 for ground debugging using time instead of altitude\n"
		"\n"
		";Chirp Settings\n"
		"chirp_control:                1   ; Set to 1 to enable chirp control. Chirp always starts in the exact middle of start freq and end freq and goes up or down from here.\n"
		"chirp_control_min_freq:       500 ; [Hz] Lowest chirp start frequency\n"
		"chirp_control_max_freq:       1500; [Hz] Highest chirp end frequency\n"
		"chirp_control_rate:           100 ;[0.01Hz] Sets rate of chirps. Value of 100 equals 1Hz.\n"
		"chirp_control_min_duration:   100 ; [ms] Minimum duration of a chirp\n"
		"chirp_control_max_duration:   900 ; [ms] Maximum duration of a chirp. Make sure not to go higher than allowed by the rate of chirps. (If rate of chirps is at 1Hz, max duration must be below 1000ms to make sense)\n";

void FS_Config_Init(void)
{
	config.model         = FS_CONFIG_MODEL_AIRBORNE_2G;
	config.rate          = 200;

	config.mode          = FS_CONFIG_MODE_GLIDE_RATIO;
	config.min           = 0;
	config.max           = 300;
	config.limits        = 1;
	config.volume        = 2;

	config.mode_2        = FS_CONFIG_MODE_CHANGE_IN_VALUE_1;
	config.min_2         = 300;
	config.max_2         = 1500;
	config.min_rate      = FS_CONFIG_RATE_ONE_HZ;
	config.max_rate      = 5 * FS_CONFIG_RATE_ONE_HZ;
	config.flatline      = 0;

	config.chirp_control 		 		=	0;		//Set to 1 to activate chirp control
	config.chirp_control_min_freq 		=	500;	//[Hz]
	config.chirp_control_max_freq 		=	1500;	//[Hz]
	config.chirp_control_rate 	 		=	100; 	//[0.01Hz] (100 => 1Hz)
	config.chirp_control_min_duration 	=	100; 	//[ms]
	config.chirp_control_max_duration 	=	900; 	//[ms]


	//################## MK: whole next block added for navMK
	config.pull_lat 		= 0;
	config.pull_lon 		= 0;
	config.pull_alt 		= 0;
	config.leg1_heading  	= 0;
	config.leg2_heading 	= 0;
	config.leg3_heading 	= 0;
	config.navMK_numLegs	= 3;
	config.av_glide_ratio	= 0;
	config.exit_altitude 	= 4000;	//500m below the flysight will play the "approaching exit altitude" sound and start to calculate the leg vectors. Once the pull point is reachable according to the vector calculations, "pull point reachable" will be played once.
	config.navMK_jump_threshold = 75;	//threshold in downward velocity to detect exit
	config.navMK_preturn_alert = 10;	//Switch to next leg if only a few meters are left in current leg. (for a 45° turn, switch 450m before if this setting is 10m/deg)
	config.navMK_Ground_Debug=0;

	//################## MK: whole next block added for WS Performance competition
	config.WScomp_compwindow_top	= 2500;	//[m] Top of the competition window (2500m for FAI)
	config.WScomp_compwindow_bottom	= 1500;	//[m] Bottom of the competition window (1500m for FAI)
	config.WScomp_exit_max			= 3350;	//[m] Maximum exit altitude for jump to count (3353m for FAI)
	config.WScomp_exit_min			= 3200;	//[m] Minimum exit altitude for jump to count (3200m for FAI)
	config.WScomp_valwindow_speed	= 10;	//[m/s] Vertical speed to be exceeded to start validation window (10m/s for FAI)
	config.WScomp_valwindow_delay	= 9;	//[s] Delay between Exceeding vertical speed and actual start of validation window (9s for FAI).
	config.WScomp_GRP_lat			= 0;	//[0.00001°] Ground Reference point Latitude. Used in combination with position at start of validation window to calculate designated flight path.
	config.WScomp_GRP_lon			= 0;	//[0.00001°] Ground Reference point Longitude. Used in combination with position at start of validation window to calculate designated flight path.
	config.WScomp_DFP_width			= 600;	//[m] Total width of designated flight path (600m for FAI)
	config.WScomp_Ground_Debug 		= 0;	//used to debug on ground

	config.sp_rate       = 0;
	config.sp_volume     = 0;

	config.num_speech    = 0;

	config.threshold     = 1000;
	config.hThreshold    = 0;

	config.use_sas       = 1;
	config.tz_offset     = 0;

	config.init_mode     = 0;
	*(config.init_filename) = '\0';

	config.alarm_window_above = 0;
	config.alarm_window_below = 0;
	config.dz_elev       = 0;

	config.num_alarms    = 0;

	config.alt_units     = FS_CONFIG_UNITS_FEET;
	config.alt_step      = 0;

	config.num_windows   = 0;

	config.enable_audio   = 1;
	config.enable_logging = 1;
	config.enable_vbat    = 1;
	config.enable_mic     = 1;
	config.enable_imu     = 1;
	config.enable_gnss    = 1;
	config.enable_baro    = 1;
	config.enable_hum     = 1;
	config.enable_mag     = 1;
	config.ble_tx_power   = 25;
	config.enable_raw     = 1;
	config.enable_mklog	  = 1;	//create MKlogfile in each folder
	config.cold_start     = 0;

	config.baro_odr       = 2;
	config.hum_odr        = 1;
	config.mag_odr        = 0;
	config.accel_odr      = 1;
	config.accel_fs       = 1;
	config.gyro_odr       = 1;
	config.gyro_fs        = 3;

	config.lat            = 0;
	config.lon            = 0;
	config.bearing        = 0;
	config.end_nav        = 0;
	config.max_dist       = 10000;
	config.min_angle      = 5;
}

FS_Config_Result_t FS_Config_Read(const char *filename)
{
	char    buffer[100];
	size_t  len;

	char    *name;
	char    *result;
	int32_t val;

	uint8_t flags = 0;

	if (f_open(&configFile, filename, FA_READ) != FR_OK)
		return FS_CONFIG_ERR;

	while (!f_eof(&configFile))
	{
		f_gets(buffer, sizeof(buffer), &configFile);

		len = strcspn(buffer, ";");
		buffer[len] = '\0';

		name = strtok(buffer, " \r\n\t:");
		if (name == 0) continue ;

		result = strtok(0, " \r\n\t:");
		if (result == 0) continue ;

		val = atol(result);

		#define HANDLE_VALUE(s,w,r,t) \
			if ((t) && !strcmp(name, (s))) { (w) = (r); }
			//s: Name of value in config File
			//w: variable to be set
			//r: value to be used to set variable
			//t: condition of value to be met (if condition does not match, nothing is set)

		HANDLE_VALUE("Model",     config.model,        val, val >= 0 && val <= 8);
		HANDLE_VALUE("Rate",      config.rate,         val, val >= 40 && val <= 1000);
		HANDLE_VALUE("Mode",      config.mode,         val, (val >= 0 && val <= 7) || (val == 11) || (val == 13) || (val == 14) || (val == 15) || (val == 16)); //MK: Added 13 FS_CONFIG_MODE_MK and 14 FS_CONFIG_MODE_NAVMK and 15 FS_CONFIG_MODE_WSCOMP and 16 FS_CONFIG_MODE_MKSPEED
		HANDLE_VALUE("Min",       config.min,          val, TRUE);
		HANDLE_VALUE("Max",       config.max,          val, TRUE);
		HANDLE_VALUE("Limits",    config.limits,       val, val >= 0 && val <= 2);
		HANDLE_VALUE("Volume",    config.volume,       8 - val, val >= 0 && val <= 8);
		HANDLE_VALUE("Mode_2",    config.mode_2,       val, (val >= 0 && val <= 9) || (val == 11) || (val == 13) || (val == 14) || (val == 15) || (val == 16)) ; //MK: Added 13 FS_CONFIG_MODE_MK and 14 FS_CONFIG_MODE_NAVMK and 15 FS_CONFIG_MODE_WSCOMP and 16 FS_CONFIG_MODE_MKSPEED
		HANDLE_VALUE("Min_Val_2", config.min_2,        val, TRUE);
		HANDLE_VALUE("Max_Val_2", config.max_2,        val, TRUE);
		HANDLE_VALUE("Min_Rate",  config.min_rate,     val * FS_CONFIG_RATE_ONE_HZ / 100, val >= 0);
		HANDLE_VALUE("Max_Rate",  config.max_rate,     val * FS_CONFIG_RATE_ONE_HZ / 100, val >= 0);
		HANDLE_VALUE("Flatline",  config.flatline,     val, val == 0 || val == 1);

		//################## MK: whole next block added for chirp control
		HANDLE_VALUE("chirp_control",    config.chirp_control,       					val, (val == 0 || val == 1)) ; 	//MK: Added for Chirp Control
		HANDLE_VALUE("chirp_control_min_freq", config.chirp_control_min_freq,        	val, (val >= 200 || val <= 10000));						//MK: Added for Chirp Control
		HANDLE_VALUE("chirp_control_max_freq", config.chirp_control_max_freq,        	val, (val >= 200 || val <= 10000));						//MK: Added for Chirp Control
		HANDLE_VALUE("chirp_control_rate", config.chirp_control_rate,    						val * FS_CONFIG_RATE_ONE_HZ / 100, val >= 0);			//MK: Added for Chirp Control
		HANDLE_VALUE("chirp_control_min_duration", config.chirp_control_min_duration,    		val, (val >= 10 || val <= 2000));						//MK: Added for Chirp Control
		HANDLE_VALUE("chirp_control_max_duration", config.chirp_control_max_duration,   val, (val >= 10 || val <= 2000));						//MK: Added for Chirp Control

		//################## MK: whole next block added for navMK
		HANDLE_VALUE("navMK_Pull_Lat", config.pull_lat,    val, (val >= -90*NAVMK_WGS84_MULTIPLIER && val <= 90*NAVMK_WGS84_MULTIPLIER));
		HANDLE_VALUE("navMK_Pull_Lon", config.pull_lon,    val, (val >= -180*NAVMK_WGS84_MULTIPLIER && val <= 180*NAVMK_WGS84_MULTIPLIER));
		HANDLE_VALUE("navMK_Pull_Alt", config.pull_alt,    val, TRUE);
		HANDLE_VALUE("navMK_Leg1", config.leg1_heading,    val, (val >= 0 && val <= 360));
		HANDLE_VALUE("navMK_Leg2", config.leg2_heading,    val, (val >= 0 && val <= 360));
		HANDLE_VALUE("navMK_Leg3", config.leg3_heading,    val, (val >= 0 && val <= 360));
		HANDLE_VALUE("navMK_numLegs", config.navMK_numLegs, val, (val >= 0 && val <= 3));
		HANDLE_VALUE("navMK_Average_Glide_Ratio", config.av_glide_ratio, val, TRUE);
		HANDLE_VALUE("navMK_exit_altitude", config.exit_altitude, val, (val>=1000 && val <= 12000));
		HANDLE_VALUE("navMK_Ground_Debug", config.navMK_Ground_Debug, val, (val == 0 || val == 1));
		HANDLE_VALUE("navMK_jump_threshold", config.navMK_jump_threshold,    val, (val >= 0 && val <= 255));
		HANDLE_VALUE("navMK_preturn_alert", config.navMK_preturn_alert,    val, (val >= 0 && val <= 255));

		//################## MK: whole next block added for WS Performance competition (lane keeping navigation)
		HANDLE_VALUE("WScomp_compwindow_top", config.WScomp_compwindow_top,    val, (val >= 0 && val <= 10000));
		HANDLE_VALUE("WScomp_compwindow_bottom", config.WScomp_compwindow_bottom,    val, (val >= 0 && val <= 10000));
		HANDLE_VALUE("WScomp_exit_max", config.WScomp_exit_max,    val, (val >= 0 && val <= 10000));
		HANDLE_VALUE("WScomp_exit_min", config.WScomp_exit_min,    val, (val >= 0 && val <= 10000));
		HANDLE_VALUE("WScomp_valwindow_speed", config.WScomp_valwindow_speed,    val, (val >= 0 && val <= 100));
		HANDLE_VALUE("WScomp_valwindow_delay", config.WScomp_valwindow_delay,    val, (val >= 0 && val <= 100));
		HANDLE_VALUE("WScomp_GRP_lat", config.WScomp_GRP_lat,    val, (val >= -90*WSCOMP_WGS84_MULTIPLIER && val <= 90*WSCOMP_WGS84_MULTIPLIER));
		HANDLE_VALUE("WScomp_GRP_lon", config.WScomp_GRP_lon,    val, (val >= -180*WSCOMP_WGS84_MULTIPLIER && val <= 180*WSCOMP_WGS84_MULTIPLIER));
		HANDLE_VALUE("WScomp_DFP_width", config.WScomp_DFP_width,    val, (val >= 0 && val <= 10000));
		HANDLE_VALUE("WScomp_Ground_Debug", config.WScomp_Ground_Debug,    val, (val == 0 || val == 1));

		HANDLE_VALUE("Sp_Rate",   config.sp_rate,      val * 1000, val >= 0 && val <= 32);
		HANDLE_VALUE("Sp_Volume", config.sp_volume,    8 - val, val >= 0 && val <= 8);
		HANDLE_VALUE("V_Thresh",  config.threshold,    val, TRUE);
		HANDLE_VALUE("H_Thresh",  config.hThreshold,   val, TRUE);
		HANDLE_VALUE("Use_SAS",   config.use_sas,      val, val == 0 || val == 1);
		HANDLE_VALUE("Window",    config.alarm_window_above, val * 1000, TRUE);
		HANDLE_VALUE("Window",    config.alarm_window_below, val * 1000, TRUE);
		HANDLE_VALUE("Win_Above", config.alarm_window_above, val * 1000, TRUE);
		HANDLE_VALUE("Win_Below", config.alarm_window_below, val * 1000, TRUE);
		HANDLE_VALUE("DZ_Elev",   config.dz_elev,      val * 1000, TRUE);
		HANDLE_VALUE("TZ_Offset", config.tz_offset,    val, TRUE);
		HANDLE_VALUE("Init_Mode", config.init_mode,    val, val >= 0 && val <= 2);
		HANDLE_VALUE("Alt_Units", config.alt_units,    val, val >= 0 && val <= 1);
		HANDLE_VALUE("Alt_Step",  config.alt_step,     val, val >= 0);

		HANDLE_VALUE("Enable_Audio",   config.enable_audio,   val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Logging", config.enable_logging, val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Vbat",    config.enable_vbat,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Mic",     config.enable_mic,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Imu",     config.enable_imu,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Gnss",    config.enable_gnss,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Baro",    config.enable_baro,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Hum",     config.enable_hum,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Mag",     config.enable_mag,     val, val == 0 || val == 1);
		HANDLE_VALUE("Ble_Tx_Power",   config.ble_tx_power,   val, val >= 0 || val <= 31);
		HANDLE_VALUE("Enable_Raw",     config.enable_raw,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_MKlog",   config.enable_mklog,   val, val == 0 || val == 1);	//MK: Added for MK Debug Log
		HANDLE_VALUE("Cold_Start",     config.cold_start,     val, val == 0 || val == 1);

		HANDLE_VALUE("Baro_ODR",  config.baro_odr,     val, val >= 0 && val <= 7);
		HANDLE_VALUE("Hum_ODR",   config.hum_odr,      val, val >= 0 && val <= 3);
		HANDLE_VALUE("Mag_ODR",   config.mag_odr,      val, val >= 0 && val <= 3);
		HANDLE_VALUE("Accel_ODR", config.accel_odr,    val, val >= 0 && val <= 11);
		HANDLE_VALUE("Accel_FS",  config.accel_fs,     val, val >= 0 && val <= 3);
		HANDLE_VALUE("Gyro_ODR",  config.gyro_odr,     val, val >= 0 && val <= 10);
		HANDLE_VALUE("Gyro_FS",   config.gyro_fs,      val, val >= 0 && val <= 3);

		HANDLE_VALUE("Lat",       config.lat,          val, val >= -900000000 && val <= 900000000);
		HANDLE_VALUE("Lon",       config.lon,          val, val >= -1800000000 && val <= 1800000000);
		HANDLE_VALUE("Bearing",   config.bearing,      val, val >= 0 && val <= 360);
		HANDLE_VALUE("End_Nav",   config.end_nav,      val * 1000, TRUE);
		HANDLE_VALUE("Max_Dist",  config.max_dist,     val, val >= 0 && val <= 10000);
		HANDLE_VALUE("Min_Angle", config.min_angle,    val, val >= 0 && val <= 360);

		#undef HANDLE_VALUE

		if (!strcmp(name, "Init_File"))
		{
			result[8] = '\0';
			strncpy(config.init_filename, result, sizeof(config.init_filename));
		}

		if (!strcmp(name, "Alarm_Elev") && config.num_alarms < FS_CONFIG_MAX_ALARMS)
		{
			if (!(flags & CONFIG_FIRST_ALARM))
			{
				config.num_alarms = 0;
				flags |= CONFIG_FIRST_ALARM;
			}

			++config.num_alarms;
			config.alarms[config.num_alarms - 1].elev = val * 1000;
			config.alarms[config.num_alarms - 1].type = 0;
			config.alarms[config.num_alarms - 1].filename[0] = '\0';
		}
		if (!strcmp(name, "Alarm_Type") && config.num_alarms <= FS_CONFIG_MAX_ALARMS)
		{
			config.alarms[config.num_alarms - 1].type = val;
		}
		if (!strcmp(name, "Alarm_File") && config.num_alarms <= FS_CONFIG_MAX_ALARMS)
		{
			result[8] = '\0';
			strncpy(config.alarms[config.num_alarms - 1].filename, result,
					sizeof(config.alarms[config.num_alarms - 1].filename));
		}

		if (!strcmp(name, "Win_Top") && config.num_windows < FS_CONFIG_MAX_WINDOWS)
		{
			if (!(flags & CONFIG_FIRST_WINDOW))
			{
				config.num_windows = 0;
				flags |= CONFIG_FIRST_WINDOW;
			}

			++config.num_windows;
			config.windows[config.num_windows - 1].top = val * 1000;
		}
		if (!strcmp(name, "Win_Bottom") && config.num_windows <= FS_CONFIG_MAX_WINDOWS)
		{
			config.windows[config.num_windows - 1].bottom = val * 1000;
		}

		if (!strcmp(name, "Sp_Mode") && config.num_speech < FS_CONFIG_MAX_SPEECH)
		{
			if (!(flags & CONFIG_FIRST_SPEECH))
			{
				config.num_speech = 0;
				flags |= CONFIG_FIRST_SPEECH;
			}

			++config.num_speech;
			config.speech[config.num_speech - 1].mode = val;
			config.speech[config.num_speech - 1].units = FS_CONFIG_UNITS_MPH;
			config.speech[config.num_speech - 1].decimals = 1;
		}
		if (!strcmp(name, "Sp_Units") && config.num_speech <= FS_CONFIG_MAX_SPEECH)
		{
			config.speech[config.num_speech - 1].units = val;
		}
		if (!strcmp(name, "Sp_Dec") && config.num_speech <= FS_CONFIG_MAX_SPEECH)
		{
			config.speech[config.num_speech - 1].decimals = val;
		}
	}

	f_close(&configFile);

	return FS_CONFIG_OK;
}

FS_Config_Result_t FS_Config_Write(const char *filename)
{
	FRESULT res;

	res = f_open(&configFile, filename, FA_WRITE|FA_CREATE_ALWAYS);
	if (res != FR_OK) return FS_CONFIG_ERR;

	f_puts(defaultConfig, &configFile);

	f_close(&configFile);

	return FS_CONFIG_OK;
}

const FS_Config_Data_t *FS_Config_Get(void)
{
	return &config;
}
