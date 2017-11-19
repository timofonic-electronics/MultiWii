#ifndef MULTIWII_H_
#define MULTIWII_H_

#define  VERSION        241
//This allow sync with GUI
#define  NAVI_VERSION   7
#include "types.h"
#include "Alarms.h"

#define MINCHECK 1100
#define MAXCHECK 1900

extern volatile unsigned long timer0_overflow_count;

extern const char pidnames[];
extern const char boxnames[];
extern const uint8_t boxids[];

extern uint32_t currentTime;
extern uint16_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t  magHold,headFreeModeHold;
extern uint8_t  vbatMin;
extern uint8_t  rcOptions[CHECKBOXITEMS];
extern int32_t  AltHold;
extern int16_t  sonarAlt;
extern int16_t  BaroPID;
extern int16_t  errorAltitudeI;

extern int16_t  i2c_errors_count;
extern uint8_t alarmArray[ALRM_FAC_SIZE];
extern global_conf_t global_conf;

extern imu_t imu;
extern analog_t analog;
extern alt_t alt;
extern att_t att;
#ifdef LOG_PERMANENT
extern plog_t plog;
#endif

extern int16_t debug[4];

extern conf_t conf;
extern int16_t  annex650_overrun_count;
extern flags_struct_t f;
extern uint16_t intPowerTrigger1;

extern int16_t gyroZero[3];
extern int16_t angle[2];


#if BARO
  extern int32_t baroPressure;
  // temp in 0.01 deg
  extern int16_t baroTemperature;
  extern int32_t baroPressureSum;
#endif

extern int16_t axisPID[3];
extern int16_t motor[8];
extern int16_t servo[8];

extern int16_t failsafeEvents;
extern volatile int16_t failsafeCnt;

extern int16_t rcData[RC_CHANS];
extern int16_t rcSerial[8];
extern int16_t rcCommand[4];
extern uint8_t rcSerialCount;
extern int16_t lookupPitchRollRC[5];
extern uint16_t lookupThrottleRC[11];

#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
// index into pMeter[] for sum
#define PMOTOR_SUM 8
// we use [0:7] for eight motors,one extra for sum
extern uint32_t pMeter[PMOTOR_SUM + 1];
// dummy to satisfy the paramStruct logic in ConfigurationLoop()
extern uint8_t pMeterV;
// we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
extern uint32_t pAlarm;
// last known current
extern uint16_t powerValue;
#endif

#if defined(LCD_TELEMETRY)
  extern uint8_t telemetry;
  extern uint8_t telemetry_auto;
#endif
#ifdef LCD_TELEMETRY_STEP
  extern char telemetryStepSequence[];
  extern uint8_t telemetryStepIndex;
#endif

#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  // highest ever cycle timen
  extern uint16_t cycleTimeMax;
  // lowest ever cycle timen
  extern uint16_t cycleTimeMin;
  // maximum value
  extern int32_t  BAROaltMax;
  // maximum speed from gps
  extern uint16_t GPS_speedMax;
  extern uint16_t powerValueMaxMAH;
  extern uint16_t wattsMax;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
  extern uint32_t armedTime;
#endif

#if GPS
// *************************************** begin GPS common variables and defines ******************************************************************

extern gps_conf_struct GPS_conf;

// the angles that must be applied for GPS correction
extern int16_t  GPS_angle[2];
extern int32_t  GPS_coord[2];
extern int32_t  GPS_home[2];
extern int32_t  GPS_hold[2];
extern int32_t  GPS_prev[2];
// Coordinates of the current poi
extern int32_t  GPS_poi[2];
// direction to the actual poi (used to set heading to poi)
extern int32_t  GPS_directionToPoi;
extern uint8_t  GPS_numSat;
// distance to home  - unit: meter
extern uint16_t GPS_distanceToHome;
// direction to home - unit: degree
extern int16_t  GPS_directionToHome;
// GPS altitude      - unit: meter
extern uint16_t GPS_altitude;
// GPS speed         - unit: cm/s
extern uint16_t GPS_speed;
// a binary toogle to distinct a GPS position update
extern uint8_t  GPS_update;
// - unit: degree*10
extern uint16_t GPS_ground_course;
extern uint32_t GPS_time;

// contains the current selected gps flight mode
extern uint8_t  GPS_mode;

// Last error situation of the nav engine
extern uint8_t NAV_error;
//State of the nav engine
extern uint8_t NAV_state;
//The mission state saved when poshold invoked during mission
extern uint8_t GPS_saved_mission_state;
//GPS_checkbox items packed into 1 byte for checking GPS mode changes
extern uint8_t prv_gps_modes;
//common timer used in navigation (contains the desired stop time in millis()
extern uint32_t nav_timer_stop;
//time in seconds to hold position
extern uint16_t nav_hold_time;
//This contains the mission step where poshold paused the runing mission.
extern uint8_t NAV_paused_at;
//The mission step which is upcoming it equals with the mission_step stored in EEPROM
extern uint8_t next_step;

//Altitude control state
#define ASCENDING           1
#define DESCENDING          -1
#define REACHED_ALT         0

// The orginal altitude used as base our new altitude during nav
extern int32_t original_altitude;
//This is the target what we want to reach
extern int32_t target_altitude;
//This is the interim value which is feeded into the althold controller
extern int32_t alt_to_hold;

extern uint32_t alt_change_timer;
extern int8_t   alt_change_flag;
extern uint32_t alt_change;
//How many loops do we have to do (alt/100 from mission step) -10 means not used jet, -1 unlimited
extern int16_t  jump_times;
//land detector variable
extern uint8_t  land_detect;


// ************************
// mission step structure
// ************************
extern mission_step_struct mission_step;

//possible action codes for a mission step
//Set waypoint
#define MISSION_WAYPOINT      1
//Poshold unlimited
#define MISSION_HOLD_UNLIM    2
//Hold for a predetermined time
#define MISSION_HOLD_TIME     3
//Return to HOME
#define MISSION_RTH           4
//Set POINT of interest
#define MISSION_SET_POI       5
//Jump to the given step (#times)
#define MISSION_JUMP          6
//Set heading to a given orientation (parameter 1 is the waym 0-359 degree
#define MISSION_SET_HEADING   7
//Land at the given position
#define MISSION_LAND          8


//Flags that this is the last step
#define MISSION_FLAG_END         0xA5
//Returned WP had an EEPROM CRC error
#define MISSION_FLAG_CRC_ERROR   0xFE
//Returned WP is the home position
#define MISSION_FLAG_HOME        0x01
//Returned WP is the hold position
#define MISSION_FLAG_HOLD        0x02
//Land when reached desired point (used in RTH)
#define MISSION_FLAG_DO_LAND     0x20
//Navigation is in progress, returned wp is home
#define MISSION_FLAG_NAV_IN_PROG 0xff

#define LAT  0
#define LON  1

extern int16_t  nav[2];


#endif

// default POSHOLD control gains
#define POSHOLD_P              .15
#define POSHOLD_I              0.0
// degrees
#define POSHOLD_IMAX           20

#define POSHOLD_RATE_P         3.4
// Wind control
#define POSHOLD_RATE_I         0.14
// try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_D         0.053
// degrees
#define POSHOLD_RATE_IMAX      20

// default Navigation PID gains
#define NAV_P                  2.5
// Wind control
#define NAV_I                  0.33
#define NAV_D                  0.083
// degrees
#define NAV_IMAX               20


// *************************************** end GPS common variables and defines ******************************************************************

extern volatile uint8_t  spekFrameFlags;
extern volatile uint32_t spekTimeLast;
extern uint8_t  spekFrameDone;

#if defined(OPENLRSv2MULTI)
// OpenLRS onboard potentiometers for P and I trim or other usages
extern uint8_t pot_P,pot_I;
#endif

// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
extern uint16_t InflightcalibratingA;
extern int16_t AccInflightCalibrationArmed;
extern uint16_t AccInflightCalibrationMeasurementDone;
extern uint16_t AccInflightCalibrationSavetoEEProm;
extern uint16_t AccInflightCalibrationActive;
#endif

#if defined(ARMEDTIMEWARNING)
  extern uint32_t  ArmedTimeWarningMicroSeconds;
#endif

#if defined(THROTTLE_ANGLE_CORRECTION)
  extern int16_t throttleAngleCorrection;
  extern int8_t  cosZ;
#endif

void annexCode();
void go_disarm();
/* MULTIWII_H_ */
#endif
