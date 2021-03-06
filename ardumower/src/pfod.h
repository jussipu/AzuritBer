/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat

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

// Android remote control (pfod App)
// For a detailed specification of the pfodApp protocol, please visit:  http://www.forward.com.au/pfod/

// example usage:
//   RemoteControl remote;
//   remote.initSerial(19200);
//   while (true){
//     remote.readSerial();
//     remote.run();
//  }


#ifndef PFOD_H
#define PFOD_H

#include <Arduino.h>
#include "drivers.h"
#include "pid.h"
#include "perimeter.h"

// pfodApp state
enum { PFOD_OFF, PFOD_MENU, PFOD_LOG_SENSORS,
       PFOD_PLOT_BAT, PFOD_PLOT_ODO2D, PFOD_PLOT_IMU, PFOD_PLOT_SENSOR_COUNTERS,
       PFOD_PLOT_SENSORS, PFOD_PLOT_PERIMETER, PFOD_PLOT_GPS, PFOD_PLOT_GPS2D,
       PFOD_PLOT_MOTOR
     };

class Robot;

class RemoteControl
{
  public:
    RemoteControl();
    void setRobot(Robot *aRobot);
    void initSerial(HardwareSerial* serialPort, uint32_t baudrate);
    bool readSerial();
    //bb10
    void processPI(String RpiCmd, float v1, float v2, float v3) ;
    void run();
  private:
    HardwareSerial* serialPort;
    Robot *robot;
    bool pfodCmdComplete;
    String pfodCmd;
    byte pfodState;
    int testmode;
    unsigned long nextPlotTime;
    //bb
    float value1;
    float value2;
    float value3;
    bool dataFromPi;
    int8_t perimeterCapture[RAW_SIGNAL_SAMPLE_SIZE];
    int perimeterCaptureIdx;
    float stringToFloat(String &s);

    // generic
    void sendYesNo(int value);
    void sendOnOff(int value);
    void sendLeftRight(int value);


    // PID slider
    void sendPIDSlider(String cmd, String title, PID &pid, double scale, float maxvalue);
    void processPIDSlider(String result, String cmd, PID &pid, double scale, float maxvalue);

    // generic slider
    void sendSlider(String cmd, String title, float value, String unit, double scale, float maxvalue, float minvalue = 0);
    void processSlider(String result, float &value, double scale);
    void processSlider(String result, long &value, double scale);
    void processSlider(String result, int &value, double scale);
    void processSlider(String result, byte &value, double scale);
    void processSlider(String result, short &value, double scale);


    // send timer menu details
    void sendTimer(ttimer_t timer);

    // main menu
    void sendMainMenu(bool update);
    void sendErrorMenu(bool update);
    void sendInfoMenu(bool update);
    void sendCommandMenu(bool update);
    void processCommandMenu(String pfodCmd);
    void sendManualMenu(bool update);
    void sendCompassMenu(bool update);
    void sendTestOdoMenu(bool update);
    void processCompassMenu(String pfodCmd);
    void processTestOdoMenu(String pfodCmd);
    void processManualMenu(String pfodCmd);
    void processSettingsMenu(String pfodCmd);

    // plotting
    void sendPlotMenu(bool update);

    // settings
    void sendSettingsMenu(bool update);
    void sendMotorMenu(bool update);
    void sendMowMenu(bool update);
    void sendBumperMenu(bool update);
    void sendDropMenu(bool update);
    void sendSonarMenu(bool update);
    void sendPerimeterMenu(bool update);
    void sendLawnSensorMenu(bool update);
    void sendImuMenu(bool update);
    void sendRemoteMenu(bool update);
    void sendBatteryMenu(bool update);
    void sendStationMenu(bool update);
    void sendOdometryMenu(bool update);
    void sendRainMenu(bool update);
    void sendGPSMenu(bool update);
    void sendRfidMenu(bool update);

    void sendDateTimeMenu(bool update);
    void sendFactorySettingsMenu(bool update);

    void sendByLaneMenu(bool update);
    void processByLaneMenu(String pfodCmd);

    void processMotorMenu(String pfodCmd);
    void processErrorMenu(String pfodCmd);
    void processMowMenu(String pfodCmd);
    void processBumperMenu(String pfodCmd);
    void processSonarMenu(String pfodCmd);
    void processPerimeterMenu(String pfodCmd);
    void processLawnSensorMenu(String pfodCmd);
    void processRainMenu(String pfodCmd);
    void processDropMenu(String pfodCmd);
    void processGPSMenu(String pfodCmd);
    void processRfidMenu(String pfodCmd);
    void processImuMenu(String pfodCmd);
    void processRemoteMenu(String pfodCmd);
    void processBatteryMenu(String pfodCmd);
    void processStationMenu(String pfodCmd);
    void processOdometryMenu(String pfodCmd);
    void processDateTimeMenu(String pfodCmd);
    void processFactorySettingsMenu(String pfodCmd);
    void processInfoMenu(String pfodCmd);

    // timer
    void sendTimerDetailMenu(int timerIdx, bool update);
    void processTimerDetailMenu(String pfodCmd);
    void sendTimerMenu(bool update);
    void processTimerMenu(String pfodCmd);

};



#endif
