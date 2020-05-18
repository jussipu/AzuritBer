/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri

  Private-use only! (you need to ask for a commercial-use)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)

*/

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>
//#include <Servo.h>  // for RC brushless contoller
#include "drivers.h"
#include "pid.h"
#include "imu.h"
#include "adcman.h"
#include "perimeter.h"
#include "gps.h"
#include "pfod.h"
#include "RunningMedian.h"

//#include "QueueList.h"
//#include <limits.h>

/*
  Generic robot class - subclass to implement concrete hardware!
*/

// code version
#define VER "1.2-Azurit-ber"

// sensors
enum
{
  SEN_PERIM_LEFT,        // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT,       // 0..MAX_PERIMETER
  SEN_PERIM_LEFT_EXTRA,  // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT_EXTRA, // 0..MAX_PERIMETER
  SEN_LAWN_FRONT,
  SEN_LAWN_BACK,
  SEN_BAT_VOLTAGE,  // Volt * 100
  SEN_CHG_CURRENT,  // Ampere * 100
  SEN_CHG_VOLTAGE,  // Volt * 100
  SEN_MOTOR_LEFT,   // 0..MAX_MOTOR_CURRENT
  SEN_MOTOR_RIGHT,  // 0..MAX_MOTOR_CURRENT
  SEN_MOTOR1_MOW,   // 0..MAX_MOW1_CURRENT
  SEN_MOTOR2_MOW,   // 0..MAX_MOW2_CURRENT
  SEN_BUMPER_LEFT,  // LOW = pressed
  SEN_BUMPER_RIGHT, // LOW = pressed
  SEN_DROP_LEFT,    // LOW = pressed - Dropsensor - Absturzsensor
  SEN_DROP_RIGHT,   // LOW = pressed - Dropsensor - Absturzsensor
  SEN_SONAR_CENTER, // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_LEFT,   // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_RIGHT,  // 0..SONAR_TRIGGER_DISTANCE
  SEN_BUTTON,       // LOW = pressed
  SEN_IMU,
  SEN_MOTOR_MOW_RPM,
  SEN_RTC,
  SEN_RAIN,
};

// actuators
enum
{
  ACT_MOTOR_LEFT,
  ACT_MOTOR_RIGHT,
  ACT_MOTOR_MOW,
  ACT_BUZZER,
  ACT_LED,
  ACT_USER_SW1,
  ACT_USER_SW2,
  ACT_USER_SW3,
  ACT_RTC,
  ACT_CHGRELAY,
  ACT_BATTERY_SW,
};

// error types
enum
{
  ERR_MOTOR_LEFT,
  ERR_MOTOR_RIGHT,
  ERR_MOTOR_MOW,
  ERR_MOW_SENSE,
  ERR_IMU_COMM,
  ERR_IMU_TILT,
  ERR_RTC_COMM,
  ERR_RTC_DATA,
  ERR_PERIMETER_TIMEOUT,
  ERR_TRACKING,
  ERR_ODOMETRY_LEFT,
  ERR_ODOMETRY_RIGHT,
  ERR_BATTERY,
  ERR_CHARGER,
  ERR_GPS_COMM,
  ERR_GPS_DATA,
  ERR_ADC_CALIB,
  ERR_IMU_CALIB,
  ERR_EEPROM_DATA,
  ERR_STUCK,
  ERR_TEMP_HIGH, // Electronics high temp
  // <---- add new error types here (NOTE: increase MAGIC to avoid corrupt EEPROM error data!)
  ERR_ENUM_COUNT,
};

// finate state machine states NEVER INSERT ALWAYS ADD TO END
enum
{
  STATE_OFF,                        // off
  STATE_REMOTE,                     // model remote control (R/C)
  STATE_FORWARD,                    // drive forward
  STATE_ROLL,                       // drive roll right/left
  STATE_REVERSE,                    // drive reverse
  STATE_CIRCLE,                     // drive circle
  STATE_ERROR,                      // error
  STATE_PERI_FIND,                  // perimeter find
  STATE_PERI_TRACK,                 // perimeter track
  STATE_PERI_ROLL,                  // perimeter roll
  STATE_PERI_REV,                   // perimeter reverse
  STATE_STATION,                    // in station
  STATE_STATION_CHARGING,           // in station charging
  STATE_STATION_CHECK,              // checks if station is present
  STATE_STATION_REV,                // charge reverse
  STATE_STATION_ROLL,               // charge roll
  STATE_STATION_FORW,               // charge forward
  STATE_MANUAL,                     // manual navigation
  STATE_ROLL_WAIT,                  // drive roll right/left
  STATE_PERI_OUT_FORW,              // outside perimeter forward driving without checkPerimeterBoundary()
  STATE_PERI_OUT_REV,               // outside perimeter reverse driving without checkPerimeterBoundary()
  STATE_PERI_OUT_ROLL,              // outside perimeter rolling driving without checkPerimeterBoundary()
  STATE_PERI_OBSTACLE_REV,          // perimeter reverse when hit obstacle
  STATE_PERI_OBSTACLE_ROLL,         // perimeter roll when hit obstacle
  STATE_PERI_OBSTACLE_FORW,         // perimeter forward after roll obstacle
  STATE_PERI_OBSTACLE_AVOID,        // perimeter circle to avoid obstacle
  STATE_NEXT_LANE_FORW,             // use after roll to reach the next lane
  STATE_PERI_OUT_STOP,              // use to stop slowly on perimeter before reverse
  STATE_PERI_OUT_LANE_ROLL1,        // roll always to 135 deg with brake
  STATE_PERI_OUT_LANE_ROLL2,        // roll one wheel to put in straight line with brake
  STATE_PERI_OUT_ROLL_TOINSIDE,     // outside perimeter rolling driving without checkPerimeterBoundary()
  STATE_WAIT_AND_REPEAT,            // use to repeat a state until something change for example find again perimeter by rolling
  STATE_FORWARD_ODO,                // drive forward odometry control straigh line and not speed in rpm
  STATE_TEST_COMPASS,               // drive roll right/left FOR A SPECIFIQUE ANGLE
  STATE_PERI_OUT_ROLL_TOTRACK,      // outside perimeter rolling driving without checkPerimeterBoundary()
  STATE_PERI_STOP_TOTRACK,          // brake when reach the wire before roll to start tracking
  STATE_AUTO_CALIBRATE,             // wait until the drift is stopped and adapt the gyro and compass
  STATE_ROLL_TO_FIND_YAW,           // try to find the compass yaw = yawheading
  STATE_TEST_MOTOR,                 // use to test the odometry motor
  STATE_STOP_TO_FIND_YAW,           // brake before roll to find yaw
  STATE_STOP_ON_BUMPER,             // stop immediatly on bumper
  STATE_STOP_CALIBRATE,             // don't move during 15 sec for DMP autocalibration.
  STATE_SONAR_TRIG,                 // when sonar detect obstacle brake gently before reverse
  STATE_STOP_BEFORE_SPIRALE,        // brake gently before start the spirale
  STATE_MOW_SPIRALE,                // mowing a CIRCLE ARC of increaseing diameter to make the spirale
  STATE_ROTATE_RIGHT_360,           // simple mowing rotation at the beginning of the spirale
  STATE_NEXT_SPIRE,                 // go to next CIRCLE ARC in SPIRALE MODE
  STATE_ESCAPE_LANE,                // grass is high need to reduce the lenght of the lane
  STATE_PERI_STOP_TOROLL,           // use when timer start the mowing or when tracking and find RFID need to stop before roll
  STATE_ROLL_TONEXTTAG,             // use when tracking and find RFID tag the mower roll to new heading
  STATE_PERI_STOP_TO_NEWAREA,       // use when tracking and find RFID need to stop before roll to leave the area
  STATE_ROLL1_TO_NEWAREA,           // use when tracking and find RFID need to roll to leave the area
  STATE_DRIVE1_TO_NEWAREA,          // use when tracking and find RFID need to drive to leave the area
  STATE_ROLL2_TO_NEWAREA,           // use when tracking and find RFID need to roll to leave the area
  STATE_DRIVE2_TO_NEWAREA,          // use when tracking and find RFID need to drive to leave the area
  STATE_WAIT_FOR_SIG2,              // use when the area 2 wait until sender send the signal
  STATE_STOP_TO_NEWAREA,            // use to stop the mower in straight line after long distance moving with ODO and IMU
  STATE_PERI_OUT_STOP_ROLL_TOTRACK, // after the mower rool to track we need to stop the right motor because it's reverse and the track is forward
  STATE_PERI_STOP_TO_FAST_START,    // after the mower find a tag for find a new start entry point
  STATE_CALIB_MOTOR_SPEED           // we need to know how may ticks the motor can do in 1 ms to compute the maxododuration
};

// status mode
enum
{
  WAIT,
  NORMAL_MOWING,
  SPIRALE_MOWING,
  BACK_TO_STATION,
  TRACK_TO_START,
  MANUAL,
  REMOTE,
  IN_ERROR,
  IN_STATION,
  TESTING,
  WAITSIG2,
  WIRE_MOWING
};

// roll types
enum
{
  LEFT,
  RIGHT
};

// mow patterns
enum
{
  MOW_RANDOM,
  MOW_LANES,
  MOW_WIRE,
  MOW_ZIGZAG
};

//bb
// console mode
enum
{
  CONSOLE_SENSOR_COUNTERS,
  CONSOLE_SENSOR_VALUES,
  CONSOLE_PERIMETER,
  CONSOLE_OFF,
  CONSOLE_TRACKING
};

#define MAX_TIMERS 5

#define BATTERY_SW_OFF -1

class Robot
{
public:
  bool debugConsole;
  String name;
  bool developerActive;
  // --------- state machine --------------------------
  byte stateCurr;
  byte stateLast;
  byte stateNext;

  byte statusCurr;
  //byte statusLast;
  //byte statusNext;

  unsigned long stateTime;
  char *stateName();
  char *statusName();

  unsigned long stateStartTime;
  unsigned long stateEndTime;
  int idleTimeSec;
  // --------- timer ----------------------------------
  ttimer_t timer[MAX_TIMERS];
  datetime_t datetime;
  bool timerUse; // use timer?
  unsigned long nextTimeTimer;
  byte ActualRunningTimer;
  // ----- bluetooth -------------------------------------
  bool bluetoothUse; // use Bluetooth module?
  // ----- esp8266 ---------------------------------------
  bool esp8266Use; // use ESP8266 Wifi module?
  String esp8266ConfigString = "";
  // -------- mow pattern -----------------------------
  byte mowPatternCurr;
  char *mowPatternName();
  // -------- gps state -------------------------------
  GPS gps;
  bool gpsUse; // use GPS?
  bool gpsReady;
  float gpsLat;
  float gpsLon;
  float gpsX; // X position (m)
  float gpsY; // Y position (m)
  unsigned long nextTimeGPS;
  unsigned long nextTimeCheckIfStuck;
  float stuckIfGpsSpeedBelow;
  int gpsBaudrate; // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)
  int robotIsStuckCounter;
  // -------- odometry state --------------------------
  bool odometryUse;               // use odometry?
  bool twoWayOdometrySensorUse;   // free for something else
  int odometryTicksPerRevolution; // encoder ticks per one full resolution
  float odometryTicksPerCm;       // encoder ticks per cm
  float odometryWheelBaseCm;      // wheel-to-wheel distance (cm)
  bool odometryRightSwapDir;      // inverse right encoder direction?
  int odometryLeft;               // left wheel counter
  int odometryRight;              // right wheel counter
  bool odometryLeftLastState;
  bool odometryLeftLastState2;
  bool odometryRightLastState;
  bool odometryRightLastState2;
  float odometryTheta;     // theta angle (radiant)
  float odometryX;         // X map position (cm)
  float odometryY;         // Y map position (cm)
  float motorLeftRpmCurr;  // left wheel rpm
  float motorRightRpmCurr; // right wheel rpm
  unsigned long lastMotorRpmTime;
  unsigned long nextTimeOdometry;
  unsigned long nextTimeOdometryInfo;
  //bb
  int stateEndOdometryRight; // use to mesure the distance when rev roll etc ...
  int stateEndOdometryLeft;
  int stateStartOdometryLeft; // use to calculate the accel
  int lastStartOdometryRight;
  int lastStartOdometryLeft; // use to calculate the accel
  int stateStartOdometryRight;
  float straightLineTheta; //angle read by odometry during the last lane to verify the IMU drift
  int DistPeriOutRev;      // Distance in CM when reach perimeter
  int DistPeriObstacleRev; // Distance in CM when prei rev obstacle
  int DistPeriOutForw;     //Distance in CM after roll to accel
  int currDistToDrive;     //IMU staright line and distance control

  // -------- RC remote control state -----------------
  bool remoteUse;   // use model remote control (R/C)?
  int remoteSteer;  // range -100..100
  int remoteSpeed;  // range -100..100
  int remoteMow;    // range 0..100
  int remoteSwitch; // range 0..100
  unsigned long remoteSteerLastTime;
  unsigned long remoteSpeedLastTime;
  unsigned long remoteMowLastTime;
  unsigned long remoteSwitchLastTime;
  bool remoteSteerLastState;
  bool remoteSpeedLastState;
  bool remoteMowLastState;
  bool remoteSwitchLastState;
  unsigned long nextTimeRTC;
  String rfidTagFind;
  bool rfidUse;

  // --------- wheel motor state ----------------------------
  // wheel motor speed ( <0 backward, >0 forward); range -motorSpeedMaxRpm..motorSpeedMaxRpm
  //bb
  int motorLeftChange;
  int motorRightChange;

  int motorAccel;              // motor wheel acceleration (warning: do not set too high)
  int motorSpeedMaxRpm;        // motor wheel max RPM
  int motorSpeedMaxPwm;        // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
  int motorInitialSpeedMaxPwm; // motor Initial wheel max Pwm
  float motorPowerMax;         // motor wheel max power (Watt)
  PID motorLeftPID;            // motor left wheel PID controller
  PID motorRightPID;           // motor right wheel PID controller
  float motorSenseRightScale;  // motor right sense scale (mA=(ADC-zero)/scale)
  float motorSenseLeftScale;   // motor left sense scale  (mA=(ADC-zero)/scale)
  /*
        int motorRollTimeMax ;  // max. roll time (ms)
        int motorRollTimeMin  ; // min. roll time (ms)
        int motorReverseTime ;  // max. reverse time (ms)
    */

  int motorRollDegMax;
  int motorRollDegMin;
  //int DistPeriOutRev;

  unsigned long motorForwTimeMax; // max. forward time (ms) / timeout
  float motorBiDirSpeedRatio1;    // bidir mow pattern speed ratio 1
  float motorBiDirSpeedRatio2;    // bidir mow pattern speed ratio 2
  bool motorRightSwapDir;         // inverse right motor direction?
  bool motorLeftSwapDir;          // inverse left motor direction?
  int motorLeftSpeedRpmSet;       // set speed
  int motorRightSpeedRpmSet;
  //bb
  int motorOdoAccel;

  int motorLeftPWMCurr; // current speed
  int motorRightPWMCurr;
  //float motorLeftPWMCurr ; // current speed
  //float motorRightPWMCurr ;
  int motorRightSenseADC;
  int motorLeftSenseADC;
  float motorLeftSenseCurrent;
  float motorRightSenseCurrent;
  float motorLeftPower; // motor power (range 0..MAX_MOTOR_POWER)
  float motorRightPower;
  int motorPowerIgnoreTime;
  int motorZeroSettleTime;   // how long (ms) to wait for motor to settle at zero speed
  int motorLeftSenseCounter; // motor current counter
  int motorRightSenseCounter;
  unsigned long nextTimeMotorSense;
  unsigned long lastSetMotorSpeedTime;
  unsigned long motorLeftZeroTimeout;
  unsigned long motorRightZeroTimeout;
  bool rotateLeft;
  unsigned long nextTimeRotationChange;
  unsigned long nextTimeSendTagToPi;
  unsigned long nextTimeMotorControl;
  unsigned long nextTimeMotorImuControl;

  //bb
  int motorRightOffsetFwd;
  int motorRightOffsetRev;
  int motorTickPerSecond;

  unsigned long nextTimeMotorOdoControl;
  unsigned long nextTimePidCompute;
  unsigned long accelDurationRight;
  unsigned long accelDurationLeft;
  unsigned long movingTimeLeft;
  unsigned long movingTimeRight;
  unsigned long nextTimeMotorPerimeterControl;
  unsigned long nextTimeMotorMowControl;
  int lastMowSpeedPWM;
  unsigned long lastSetMotorMowSpeedTime;
  unsigned long nextTimeCheckCurrent;
  unsigned long lastTimeMotorMowStuck;
  int leftSpeed;
  int rightSpeed;
  int PwmLeftSpeed;
  int PwmRightSpeed;
  int SpeedOdoMaxLeft;
  int SpeedOdoMaxRight;
  int SpeedOdoMin; //Minimum PWM speed to have the wheel turn
  int SpeedOdoMax; //Maximum PWM speed to small mouvment

  float OdoStartBrakeLeft;
  float OdoStartBrakeRight;
  float MaxOdoStateDuration;
  float PrevStateOdoDepassLeft;
  float PrevStateOdoDepassRight;
  float motorLeftSpeedDivider;
  bool UseAccelRight;
  bool UseBrakeRight;
  bool UseAccelLeft;
  bool UseBrakeLeft;
  bool odoLeftRightCorrection;
  int AngleRotate;
  int newtagRotAngle1;
  int newtagRotAngle2;
  int newtagDistance1;
  int newtagDistance2;
  int foobar;
  //unsigned long stateMaxiTime;  // maybe safety if the odometry is not reach in this time error

  int angleCorresp;
  int accelDuration;
  byte RollToInsideQty; //use to stop if roll non stop

  // -------- mower motor state -----------------------
  int motorMowRpmCounter; // mower motor speed state
  bool secondMowMotor;    // second mow motor
  bool motorMowRpmLastState;
  bool motorMowEnable;   // motor can be temporary disabled if stucked etc. with this
  bool motorMowForceOff; // user switch for mower motor on/off has highest priority
  // bool ignoreRfidTag ; // use to stay on wire when mow perimeter
  bool highGrassDetect;            //detect that the mow motor is on high load so high grass
  float triggerMotorMowHightGrass; // motor mower percent of power vs power Max  trigger to start spirale or half lane
  // bool motorMowEnableOverride ; // user switch for mower motor on/off has highest priority if true the motor is stop
  // mower motor sppeed; range 0..motorMowSpeedMaxPwm
  float motorMowAccel;     // motor mower acceleration (warning: do not set too high)
  int motorMowSpeedMaxPwm; // motor mower max PWM
  float motorMowPowerMax;  // motor mower max power (Watt)
  //char motorMowModulate  ;      // motor mower cutter modulation?
  //bb 8
  byte spiraleNbTurn; //count the number of revolution of the spirale (10 revolutions for example before stop)
  byte halfLaneNb;    //count the number of lane same as spirale (10  for example before stop)

  bool motorMowModulate;     // motor mower cutter modulation?
  int motorMowRPMSet;        // motor mower RPM (only for cutter modulation)
  float motor1MowSenseScale; // motor 1 mower sense scale (mA=(ADC-zero)/scale)
  float motor2MowSenseScale; // motor 2 mower sense scale (mA=(ADC-zero)/scale)
  PID motorMowPID;           // motor mower RPM PID controller
  int motorMowSpeedPWMSet;
  int motorMowPWMCurr; // current speed
  int motor1MowSenseADC;
  int motor2MowSenseADC;
  float motor1MowSenseCurrent; // motor 1 mA
  float motor2MowSenseCurrent; // motor 2 mA
  float motor1MowSense;        // motor 1 power (range 0..MAX_MOW_POWER)
  float motor2MowSense;        // motor 2 power (range 0..MAX_MOW_POWER)
  int motor1MowSenseCounter;
  int motor2MowSenseCounter;
  int motorMowSenseErrorCounter;
  int motorMowRpmCurr; // motor rpm (range 0..MOW_RPM)
  unsigned long lastMotorMowRpmTime;

  // --------- bumper state ---------------------------
  // bumper state (true = pressed)
  bool bumperUse; // has bumpers?
  bool tiltUse;   // has tilt sensor?
  bool tilt;
  int bumperLeftCounter;
  bool bumperLeft;
  int bumperRightCounter;
  bool bumperRight;
  unsigned long nextTimeBumper;
  // --------- drop state ---------------------------
  // bumper state (true = pressed)                                                                                                  // Dropsensor - Absturzsensor vorhanden ?
  bool dropUse;               // has drops?                                                                                           // Dropsensor - Absturzsensor Zähler links
  int dropLeftCounter;        // Dropsensor - Absturzsensor
  bool dropLeft;              // Dropsensor - Absturzsensor links betätigt ?
  int dropRightCounter;       // Dropsensor - Absturzsensor
  bool dropRight;             // Dropsensor - Absturzsensor rechts betätigt ?
  unsigned long nextTimeDrop; // Dropsensor - Absturzsensor
  bool dropcontact;           // contact 0-openers 1-closers                                                                                 // Dropsensor Kontakt 0 für Öffner - 1 Schließer
  // ------- IMU state --------------------------------
  IMUClass imu;
  bool imuUse;                // use IMU?
  bool stopMotorDuringCalib;  // Stop mow motor during auto calibration
  PID imuDirPID;              // direction PID controller
  PID imuRollPID;             // roll PID controller
  float imuDriveHeading;      // drive heading (IMU)
  float periFindDriveHeading; // drive heading when search for other rfid tag or go to station
  float remoteDriveHeading;   // drive heading  when rfid tag to go to area2
  float newtagRotAngle1Radian;

  float imuRollHeading; // roll heading  (IMU)
  byte imuRollDir;
  //point_float_t accMin;
  //point_float_t accMax;
  unsigned long nextTimeIMU;       //read IMU data
  unsigned long nextTimeCheckTilt; // check if
  unsigned long nextTimeAddYawMedian;
  //unsigned long nextTimeImuUse;
  unsigned long nextTimeToDmpAutoCalibration;
  unsigned long endTimeCalibration;
  bool needDmpAutoCalibration;
  //float lastLaneYawMedian; //use to know the last lane direction in bylaneodo mowing
  float YawActualDeg;
  byte compassRollSpeedCoeff;
  RunningMedian compassYawMedian = RunningMedian(60);
  RunningMedian accelGyroYawMedian = RunningMedian(60);
  //bb 5

  float yawToFind;
  float findedYaw;
  float yawSet1;
  float yawOppositeLane1RollRight;
  float yawOppositeLane1RollLeft;
  float yawSet2;
  float yawOppositeLane2RollRight;
  float yawOppositeLane2RollLeft;
  float yawSet3;
  float yawOppositeLane3RollRight;
  float yawOppositeLane3RollLeft;
  float yawCiblePos;
  float yawCibleNeg;
  byte laneUseNr;
  byte DistBetweenLane;
  byte maxLenghtByLane;
  byte actualLenghtByLane;
  int correctLeft;
  int correctRight;
  float YawMedianDeg;
  bool justChangeLaneDir; //
  byte actualRollDirToCalibrate;
  float prevYawCalcOdo;
  unsigned long nextTimeImuLoop;
  int delayBetweenTwoDmpAutocalib;
  int maxDurationDmpAutocalib;
  float maxDriftPerSecond;

  // ------- perimeter state --------------------------

  //bb
  PerimeterClass perimeter;

  // Perimeter perimeter;
  bool perimeterUse; // use perimeter?
  //int perimeterOutRollTimeMax ;  //free but conserve for eeprom recovery
  //int perimeterOutRollTimeMin; //free but conserve for eeprom recovery
  int perimeterOutRevTime;
  int perimeterTrackRollTime;         // perimeter tracking roll time (ms)
  int perimeterTrackRevTime;          // perimeter tracking reverse time (ms)
  PID perimeterPID;                   // perimeter PID controller
  int perimeterMag;                   // perimeter magnitude
  int perimeterMagRight;              // perimeter magnitude
  byte areaInMowing;                  //it's the area in mowing nr
  bool perimeterInside;               // is inside perimeter?
  unsigned long perimeterTriggerTime; // time to trigger perimeter transition (timeout)
  int perimeterTriggerMinSmag;        // perimeter trigger timeout (ms)
  unsigned long perimeterLastTransitionTime;
  int perimeterCounter; // counts perimeter transitions
  unsigned long nextTimePerimeter;
  int trackingPerimeterTransitionTimeOut;
  int trackingErrorTimeOut;
  bool trakBlockInnerWheel;

  //add BB
  int leftSpeedperi;
  int rightSpeedperi;
  int lastLeftSpeedperi;
  int lastRightSpeedperi;
  int uu;
  int vv;
  unsigned long lastTimeForgetWire;
  unsigned long NextTimeNormalSpeed;
  // unsigned long timeToResetSpeedPeri;
  int LastPerimeterMag;
  double CiblePeriValue;
  int MaxSpeedperiPwm;
  int ActualSpeedPeriPWM;
  unsigned long RollTimeFor45Deg;
  unsigned long circleTimeForObstacle;
  int DistPeriObstacleAvoid;
  int DistPeriObstacleForw;
  int DistPeriOutStop;
  int perimeterMagMaxValue;
  int Tempovar;
  bool lastPerimeterTrackInside; // was inside or outside
  float PeriCoeffAccel;
  float R;
  int smoothPeriMag;
  unsigned long nextTimeReadSmoothPeriMag; //use when wait for sig2

  //End add bb
  //  --------- lawn state ----------------------------
  bool lawnSensorUse; // use capacitive Sensor
  int lawnSensorCounter;
  bool lawnSensor;       // lawn capacity sensor state (true = no lawn detected)
  float lawnSensorFront; // front lawn sensor capacity (time)
  float lawnSensorFrontOld;
  float lawnSensorBack; // back lawn sensor capacity (time)
  float lawnSensorBackOld;
  unsigned long nextTimeLawnSensor;
  unsigned long nextTimeLawnSensorCheck;
  // --------- rain -----------------------------------
  bool rain;
  bool rainWS; // weather station
  bool rainUse;
  int rainCounter;
  int rainReadDelay; // rain read delay
  int wsRainData;    // WS rain data selection
  unsigned long nextTimeRain;
  // --------- sonar ----------------------------------
  // ultra sonic sensor distance-to-obstacle (cm)
  bool sonarUse; // use ultra sonic sensor?
  bool sonarLeftUse;
  bool sonarRightUse;
  bool sonarCenterUse;
  int sonarTriggerBelow; // start to reverse
  int sonarSlowBelow;    // start to slow not use but stay here to keep the compatibily with 1.08
  int sonarDistCenter;
  int sonarDistRight;
  int sonarDistLeft;
  //unsigned int sonarDistCounter ;
  //unsigned int tempSonarDistCounter ;
  unsigned long sonarObstacleTimeout;
  //unsigned long nextTimeSonar ;
  unsigned long nextTimeCheckSonar;
  byte distToObstacle; //min distance to obstacle in CM of the 3 sonars
  byte sonarToFrontDist;
  //bool sonarReduceSpeed ; // if true the mower reduce speed

  // --------- pfodApp ----------------------------------
  RemoteControl rc; // pfodApp
  unsigned long nextTimePfodLoop;
  // ----- other -----------------------------------------
  bool buttonUse;      // has digital ON/OFF button?
  bool RaspberryPIUse; //a raspberryPI is connected to USBNativeport

  unsigned long beepOnOFFDuration; //variable use for the beeper
  bool beepState;                  //for the beeper true when sound
  unsigned long nextTimeBeeper;    // use for beeper
  bool startByTimer;               // use to know if the start is initiate by timer or manual via PFOD
  int whereToStart;                // use to know where the mower need to leave the wire and start to mow
  int whereToResetSpeed;           // use with Rfid Speed to know when reset to maxpwm
  int beaconToStart;               // use to know where the mower need to leave the wire and start to mow
  byte areaToGo;                   // use to know the area where to start by timer
  //-------- Temperature humidity ------------------
  bool DHT22Use;     //for the DHT22
  bool raspiTempUse; // Use raspberry temperature
  unsigned long nextTimeReadDHT22;
  // unsigned long nextTimeReadRaspiTemp;
  float humidityDht;
  float temperatureDht;
  float raspiTemp;
  float maxTemperature; // switch to OFF when DHT reach this temp
  float raspiTempMax;   // switch to OFF when Raspberry reach this temp

  // ----- user-defined switch ---------------------------
  bool userSwitch1; // user-defined switch 1 (default value)
  bool userSwitch2; // user-defined switch 2 (default value)
  bool userSwitch3; // user-defined switch 3 (default value)
  // --------- charging -------------------------------

  bool batMonitor;               // monitor battery and charge voltage?
  float batGoHomeIfBelow;        // drive home voltage (Volt)
  float batSwitchOffIfBelow;     // switch off if below voltage (Volt)
  int batSwitchOffIfIdle;        // switch off battery if idle for minutes
  float batFactor;               // battery conversion factor
  float batChgFactor;            // battery conversion factor
  float batFull;                 // battery reference Voltage (fully charged)
  float batChargingCurrentMax;   // maximum current your charger can devliver
  float batFullCurrent;          // current flowing when battery is fully charged
  float startChargingIfBelow;    // start charging if battery Voltage is below
  unsigned long chargingTimeout; // safety timer for charging
  float chgSenseZero;            // charge current sense zero point
  float batSenseFactor;          // charge current conversion factor
  float chgSense;                // mV/A empfindlichkeit des Ladestromsensors in mV/A (Für ACS712 5A = 185)
  char chgChange;                // messwertumkehr von - nach +         1oder 0
  float batVoltage;              // battery voltage (Volt)
  // byte chgSelection     ;       // Senor Auswahl
  float batRefFactor;
  float batCapacity;            // battery capacity (mAh)
  float chgVoltage;             // charge voltage (Volt)
  float chgCurrent;             // charge current  (Ampere)
  int chgNull;                  // Nulldurchgang Ladestromsensor
  byte stationRevDist;          // charge station reverse distance cm
  byte stationRollAngle;        // charge station roll angle
  int stationForwDist;          // charge station forward distance cm
  byte stationCheckDist;        // charge station check distance cm
  bool UseBumperDock;           // bumper is pressed when docking or not
  bool autoResetActive;         // at the end of the charging all is reboot to avoid error after 1 or 2 weeks ON
  byte dockingSpeed;            // speed docking is (percent of maxspeed) when sonar detect something while tracking
  unsigned long totalDistDrive; // use to check when to leave the wire in start timer mode
  unsigned long nextTimeBattery;
  unsigned long nextTimeCheckBattery;
  unsigned long nextTimeChgRasPISendBat;   // wait before sendbat to raspi
  unsigned long delayToReadVoltageStation; //wait before read the voltage
  int statsBatteryChargingCounter;
  int statsBatteryChargingCounterTotal;
  float statsBatteryChargingCapacityTrip;
  float statsBatteryChargingCapacityTotal;
  float statsBatteryChargingCapacityAverage;
  float lastTimeBatCapacity;
  // --------- error counters --------------------------
  byte errorCounterMax[ERR_ENUM_COUNT];
  byte errorCounter[ERR_ENUM_COUNT];
  // --------- other ----------------------------------
  bool resetByWDT; // watchdog reset detection
  int loopsPerSec; // main loops per second
  int loopsPerSecCounter;
  byte buttonCounter;
  byte ledState;
  byte consoleMode;
  unsigned long nextTimeButtonCheck;
  unsigned long nextTimeInfo;
  unsigned long nextTimePrintConsole;
  byte rollDir;
  unsigned long nextTimeButton;
  unsigned long nextTimeErrorCounterReset;
  unsigned long nextTimeErrorBeep;
  unsigned long nextTimeSendErr;

  unsigned long endBeepTime; //time in millis when the beep need to stop
  int beepOnDuration;        // div by 10 duration ON
  int beepOffDuration;       // div by 10 duration OFF
  int beepfrequenceOn;       // div by 10 frequence ON
  int beepfrequenceOff;      // div by 10 frequence OFF normaly 0 but can be use to make other tone

  // ------------robot stats---------------------------
  bool statsOverride;
  bool statsMowTimeTotalStart;
  unsigned int statsMowTimeMinutesTripCounter;
  unsigned long statsMowTimeMinutesTotal;
  float statsMowTimeHoursTotal;
  int statsMowTimeMinutesTrip;
  unsigned long nextTimeRobotStats;
  byte mowPatternDuration;    // use to know when need to change, the pattern change each x minutes so byte is OK
  byte mowPatternDurationMax; // value enter into PFOD for setting in Minutes
  //bb

  // --------------------------------------------------
  Robot();
  // robot setup
  virtual void setup();
  // robot main loop
  virtual void loop();
  virtual void resetIdleTime();

  // call this from R/C control interrupt
  virtual void setRemotePPMState(unsigned long timeMicros, bool remoteSpeedState, bool remoteSteerState, bool remoteMowState, bool remoteSwitchState);

  // call this from hall sensor interrupt
  virtual void setMotorMowRPMState(bool motorMowRpmState);

  // state machine
  virtual void setNextState(byte stateNew, byte dir);

  // motor
  virtual void setMotorPWM(int pwmLeft, int pwmRight, bool useAccel);
  virtual void setMotorMowPWM(int pwm, bool useAccel);

  // GPS
  virtual void processGPSData();

  // read hardware sensor (HAL)
  //bb
  virtual int readSensor(char type) {}

  // set hardware actuator (HAL)
  virtual void setActuator(char type, int value) {}

  // settings
  virtual void deleteUserSettings();
  virtual void saveUserSettings();
  virtual void deleteRobotStats();
  virtual void newTagFind();
  virtual void autoReboot();
  virtual void rebootPi();
  // other
  // virtual void beep(int numberOfBeeps, bool shortbeep);
  virtual void printInfo(Stream &s);
  virtual void setUserSwitches();
  virtual void addErrorCounter(byte errType);
  virtual void resetErrorCounters();
  virtual void resetMotorFault(); // {}
  //bb
  virtual void setBeeper(int totalDuration, byte OnDuration, byte OffDuration, byte frequenceOn, byte frequenceOff); // Set the variable for the beeper
  //virtual void RaspberryPISendStat ();
  virtual void receivePiPfodCommand(String RpiCmd, float v1, float v2, float v3);
  virtual void printSettingSerial();

protected:
  // convert ppm time to RC slider value
  virtual int rcValue(int ppmTime);
  virtual void loadSaveErrorCounters(bool readflag);
  virtual void loadSaveUserSettings(bool readflag);
  virtual void loadSaveRobotStats(bool readflag);
  virtual void loadUserSettings();
  virtual void checkErrorCounter();

  // read sensors
  virtual void readSensors();

  // read serial
  virtual void readSerial();

  // check sensor
  virtual void checkButton();
  virtual void checkBattery();
  virtual void checkTimer();
  virtual void checkCurrent();
  virtual void checkBumpers();
  virtual void checkDrop(); // Dropsensor - Absturzsensor
  virtual void checkBumpersPerimeter();
  virtual void checkPerimeterBoundary();

  virtual void checkLawn();
  virtual void checkSonar();
  virtual void checkSonarPeriTrack();

  virtual void checkTilt();
  virtual void checkRain();
  virtual void checkTimeout();
  virtual void checkOdometryFaults();
  virtual void checkIfStuck();
  virtual void checkRobotStats();

  // motor controllers
  virtual void motorControl();

  //bb
  virtual void OdoRampCompute();
  virtual void motorControlOdo();
  virtual void motorControlPerimeter();
  virtual void readDHT22();
  virtual void motorMowControl();

  // date & time
  virtual void setDefaultTime();

  // set reverse
  virtual void reverseOrBidir(byte aRollDir);

  // other
  virtual void printRemote();
  virtual void printOdometry();
  virtual void printMenu();
  virtual void delayInfo(int ms);
  virtual void delayWithWatchdog(int ms);

  // virtual void testOdometry();
  virtual void testMotors();
  virtual void setDefaults();

  // virtual void receiveGPSTime();
  virtual void calcOdometry();
  virtual void menu();
  virtual void commsMenuBT();
  virtual void commsMenuWifi();
  virtual void commsMenuSelect();
  virtual void configureBluetooth(bool quick){};

  virtual void beeper();

  // Console helpers
  virtual void purgeConsole();
  virtual char waitCharConsole();
  virtual String waitStringConsole();
};

#endif
