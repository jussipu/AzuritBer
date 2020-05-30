#!/usr/bin/env python3

from robot import *
import sys
import serial
import pynmea2
import time
import subprocess
import pickle
import smtplib
import urllib.request
import urllib.error
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email import encoders
import os.path
import os
# from tkinter import ttk
# from threading import Thread
# from tkinter import messagebox
# from tkinter import filedialog
# import tkinter as tk
import math
# add to avoid KST plot error on path
sys.path.insert(0, '/home/pi/Documents/PiArdumower')

# CONFIG
myOS = 'Linux'
DueConnectedOnPi = True
UseWeatherStation = True
autoRecordBatCharge = False
GpsConnectedOnPi = False
NanoConnectedOnPi = False
useDebugConsole = True
useEmailAlert = True
useMqtt = True
Mqtt_MowerName = 'ArduMower'
Mqtt_Broker_IP = '192.168.1.5'
Mqtt_Port = 1883

#################################### CHECK NETWORK CONNECTION ##############################################


def checkNetwork(url_to_check):
    try:
        urllib.request.urlopen(url_to_check, timeout=2)
        return True
    except urllib.error.URLError as err:
        return False


#################################### MQTT MANAGEMENT ###############################################

#bber30 test MQTT see also line 521 for the publish message
if (useMqtt):
    import paho.mqtt.client as mqtt_client
    KEEP_ALIVE = 60
    Mqqt_client = mqtt_client.Client(client_id=Mqtt_MowerName)
    Mqqt_client.connected_flag = False  # create flag in class

    def Mqqt_on_log(Mqqt_client, userdata, level, buf):
        print("log: ", buf)

    def Mqqt_on_connect(Mqqt_client, userdata, flags, rc):
        if rc == 0:
            Mqqt_client.connected_flag = True  #set flag
            print("MQTT connected " + '\n')

        else:
            Mqqt_client.connected_flag = False
            print("MQTT Bad connection Returned Code:" + rc + '\n')

    def Mqqt_on_disconnect(Mqqt_client, userdata, rc):
        Mqqt_client.connected_flag = False
        mymower.timeToReconnectMqtt = time.time() + 120
        print("MQTT Disconnected Code:" + str(rc) + '\n')

    def Mqqt_on_publish(Mqqt_client, userdata, result):
        if (Mqqt_client.connected_flag):
            mymower.callback_id = int(result)
            print("MQTT Callback message  " + str(mymower.callback_id) + '\n')
        else:
            print("MQTT Callback error last message id  " +
                  mymower.mqtt_message_id + " return " +
                  str(mymower.callback_id) + '\n')
            mymower.callback_id = 0
            #receive a callback from the last message send before disconnect

    def Mqqt_on_message(Mqqt_client, userdata, message):
        print("Reception message MQTT..." + '\n')
        print("Topic : %s" % message.topic + " Data  : %s" % message.payload +
              '\n')
        message_txt = str((message.payload), 'utf8')
        responsetable = message_txt.split(";")
        #Here the main option to do from COMMAND topic
        # if (str(message.topic) == "Mower/COMMAND/VIDEO/"):
        #     if (message_txt == "ON"):
        #         print("Start Video streaming" + '\n')
        #         myStreamVideo.start(0)
        #     if (message_txt == 'OFF'):
        #         print("Stop Video streaming" + '\n')
        #         myStreamVideo.stop()

        if (str(message.topic) == "Mower/COMMAND/"
                or str(message.topic) == "Mower/COMMAND"):
            if (str(responsetable[0]) == "RESTARTONLYPI"):
                subprocess.Popen('/home/pi/Documents/PiArdumower/Restart.py')
            if (str(responsetable[0]) == "RESTARTALL"):
                send_pfo_message(
                    'rx',
                    '1',
                    '2',
                    '3',
                    '4',
                    '5',
                    '6',
                )
            if (str(responsetable[0]) == "HOME"):
                button_home_click()
            if (str(responsetable[0]) == "STOP"):
                button_stop_all_click()
            if (str(responsetable[0]) == "START"):
                buttonStartMow_click()
            if (str(responsetable[0]) == "MOWPATTERN"):
                #Maybe need to stop and restart to mow between change ???
                send_var_message('w', 'mowPatternCurr',
                                 '' + int(responsetable[1]) + '', '0', '0',
                                 '0', '0', '0', '0', '0')
            if (str(responsetable[0]) == "PAUSE"):
                tempVar = mymower.millis + (3600000 * int(responsetable[1]))
                send_var_message('w', 'nextTimeTimer', '' + str(tempVar) + '',
                                 '0', '0', '0', '0', '0', '0', '0')

            if (str(responsetable[0]) == "STARTTIMER"):
                send_var_message('w', 'mowPatternCurr',
                                 '' + str(responsetable[1]) + '', 'laneUseNr',
                                 '' + str(responsetable[2]) + '', 'rollDir',
                                 '' + str(responsetable[3]) + '', '0', '0',
                                 '0')
                send_var_message('w', 'whereToStart',
                                 '' + str(responsetable[4]) + '', 'areaToGo',
                                 '' + str(responsetable[5]) + '',
                                 'actualLenghtBylane',
                                 '' + str(responsetable[6]) + '', '0', '0',
                                 '0')
                send_pfo_message(
                    'rv',
                    '1',
                    '2',
                    '3',
                    '4',
                    '5',
                    '6',
                )

    #the on_log and on_publish show debug message in terminal
    Mqqt_client.on_log = Mqqt_on_log
    #Mqqt_client.on_publish = Mqqt_on_publish
    Mqqt_client.on_message = Mqqt_on_message
    Mqqt_client.on_connect = Mqqt_on_connect
    Mqqt_client.on_disconnect = Mqqt_on_disconnect

    def Mqqt_DisConnection():
        Mqqt_client.loop_stop()  #Stop loop
        Mqqt_client.disconnect()  # disconnect

    def Mqqt_Connection():
        print("PING Broker" + '\n')
        testNet = os.system("ping -c 1 -W 2000 " + Mqtt_Broker_IP)
        if (testNet == 0):
            print("Broker OK" + '\n')
            try:
                Mqqt_client.username_pw_set(username="mqtt_ardumower",
                                            password="u6@@5Buqe7*K")
                Mqqt_client.connect(host=Mqtt_Broker_IP,
                                    port=Mqtt_Port,
                                    keepalive=KEEP_ALIVE)
                Mqqt_client.subscribe("Mower/COMMAND/#")
                Mqqt_client.loop_start()
                print("MQTT Connecting Please Wait " + '\n')
                mymower.callback_id = 0
                mymower.mqtt_message_id = 0

            except:
                Mqqt_client.connected_flag = False
                print("MQTT connection failed" + '\n')
                #Mqqt_client.loop_stop()    #Stop loop
                mymower.callback_id = 0
                mymower.mqtt_message_id = 0

        else:
            print("BROKER NOT CONNECTED" + '\n')

    def sendMqtt(var_topic, var_payload):
        if (Mqqt_client.connected_flag):
            r = Mqqt_client.publish(topic=var_topic,
                                    payload=var_payload,
                                    qos=0,
                                    retain=False)
            #mymower.mqtt_message_id=int(r[1])
            #print("MQTT send message " + var_topic + " " + var_payload + '\n')

        else:
            print("MQTT not connected" + '\n')
            pass
            #print("MQTT not Connected fail to send " + str(mymower.mqtt_message_id) + " " + var_topic + " " + var_payload + '\n')

    #END ADDon For MQTT

#################################### RFID MANAGEMENT ###############################################


def find_rfid_tag():
    #mymower.lastRfidFind
    #mymower.lastRfidFind=0

    search_code = mymower.lastRfidFind
    search_status = myRobot.statusNames[mymower.status]

    mymower.newtagToDo = "Null"
    for i in range(0, len(rfid_list)):
        #if str("b'"+rfid_list[i][0]+"'")== str(search_code):
        if (str(rfid_list[i][0]) == str(search_code)) & (str(
                rfid_list[i][1]) == str(search_status)):
            mymower.newtagToDo = rfid_list[i][2]
            mymower.newtagSpeed = rfid_list[i][3]
            mymower.newtagRotAngle1 = rfid_list[i][4]
            mymower.newtagDistance1 = rfid_list[i][5]
            mymower.newtagRotAngle2 = rfid_list[i][6]
            mymower.newtagDistance2 = rfid_list[i][7]

    if (mymower.newtagToDo == "Null"):
        print('  RFID Tag : %s' % (search_code) + '\n')
        print('  Status : %s' % (search_status) + '\n')
        print('  Are not present in the database \n')

    else:
        print('RFID Tag find ToDO is: %s' % (mymower.newtagToDo) + '\n')

        if ((mymower.newtagToDo == "RTS"
             )):  #return to station from station area
            print('RFID Find faster return' + '\n')
            send_var_message('w', 'newtagRotAngle1',
                             '' + str(mymower.newtagRotAngle1) + '',
                             'motorSpeedMaxPwm',
                             '' + str(mymower.newtagSpeed) + '', '0', '0', '0',
                             '0', '0')
            send_pfo_message(
                'rz',
                '1',
                '2',
                '3',
                '4',
                '5',
                '6',
            )

        if ((mymower.newtagToDo == "FAST_START")):  #find a faster way to start
            print('RFID Find faster start' + '\n')
            send_var_message('w', 'newtagRotAngle1',
                             '' + str(mymower.newtagRotAngle1) + '',
                             'motorSpeedMaxPwm',
                             '' + str(mymower.newtagSpeed) + '', '0', '0', '0',
                             '0', '0')
            send_pfo_message(
                'ru',
                '1',
                '2',
                '3',
                '4',
                '5',
                '6',
            )

        if (mymower.newtagToDo == "SPEED"):  #find the station for example
            print('RFID change tracking speed' + '\n')
            send_var_message('w', 'newtagDistance1',
                             '' + str(mymower.newtagDistance1) + '', '0', '0',
                             '0', '0', '0', '0', '0')
            send_var_message('w', 'ActualSpeedPeriPWM',
                             '' + str(mymower.newtagSpeed) + '', '0', '0', '0',
                             '0', '0', '0', '0')

        if ((mymower.newtagToDo == "NEW_AREA")):
            if (mymower.areaToGo != mymower.areaInMowing):
                print('Go to area ---> ' + str(mymower.areaToGo) + '\n')
                send_var_message('w', 'motorSpeedMaxPwm',
                                 '' + str(mymower.newtagSpeed) + '', '0', '0',
                                 '0', '0', '0', '0', '0')
                send_var_message('w', 'newtagRotAngle1',
                                 '' + str(mymower.newtagRotAngle1) + '', '0',
                                 '0', '0', '0', '0', '0', '0')
                send_var_message('w', 'newtagRotAngle2',
                                 '' + str(mymower.newtagRotAngle2) + '', '0',
                                 '0', '0', '0', '0', '0', '0')
                send_var_message('w', 'newtagDistance1',
                                 '' + str(mymower.newtagDistance1) + '', '0',
                                 '0', '0', '0', '0', '0', '0')
                send_var_message('w', 'newtagDistance2',
                                 '' + str(mymower.newtagDistance2) + '', '0',
                                 '0', '0', '0', '0', '0', '0')
                send_pfo_message(
                    'ry',
                    '1',
                    '2',
                    '3',
                    '4',
                    '5',
                    '6',
                )

            else:  #we are already in the mowing area so return to station from other area
                mymower.areaToGo = 1
                print('Return to Station area ---> ' + '\n')
                send_var_message('w', 'motorSpeedMaxPwm',
                                 '' + str(mymower.newtagSpeed) + '', '0', '0',
                                 '0', '0', '0', '0', '0')
                send_var_message('w', 'newtagRotAngle1',
                                 '' + str(mymower.newtagRotAngle1) + '', '0',
                                 '0', '0', '0', '0', '0', '0')
                send_var_message('w', 'newtagRotAngle2',
                                 '' + str(mymower.newtagRotAngle2) + '', '0',
                                 '0', '0', '0', '0', '0', '0')
                send_var_message('w', 'newtagDistance1',
                                 '' + str(mymower.newtagDistance1) + '', '0',
                                 '0', '0', '0', '0', '0', '0')
                send_var_message('w', 'newtagDistance2',
                                 '' + str(mymower.newtagDistance2) + '', '0',
                                 '0', '0', '0', '0', '0', '0')
                send_var_message('w', 'areaToGo', '1', '0', '0', '0', '0', '0',
                                 '0', '0')
                send_pfo_message(
                    'ry',
                    '1',
                    '2',
                    '3',
                    '4',
                    '5',
                    '6',
                )


#################################### READ RASPBERRY PI TEMPERATURE #########################################


def readRasPiTemp():
    if mymower.raspiTempUse:
        mymower.raspiTemp = os.popen(
            '/opt/vc/bin/vcgencmd measure_temp | cut -c6-9').read()
        send_var_message('w', 'raspiTemp',
                         '' + str(float(mymower.raspiTemp)) + '', '0', '0',
                         '0', '0', '0', '0', '0')


###################################### READ RASPBERRY PI UPTIME ############################################


def uptime():
    try:
        f = open("/proc/uptime")
        contents = f.read().split()
        f.close()
    except:
        return "Cannot open uptime file: /proc/uptime"

    total_seconds = float(contents[0])

    # Helper vars:
    MINUTE = 60
    HOUR = MINUTE * 60
    DAY = HOUR * 24

    # Get the days, hours, etc:
    days = int(total_seconds / DAY)
    hours = int((total_seconds % DAY) / HOUR)
    minutes = int((total_seconds % HOUR) / MINUTE)
    seconds = int(total_seconds % MINUTE)

    # Build up the pretty string (like this: "N days, N hours, N minutes, N seconds")
    string = ""
    if days > 0:
        string += str(days) + " " + (days == 1 and "day" or "days") + ", "
    if len(string) > 0 or hours > 0:
        string += str(hours) + " " + (hours == 1 and "hour" or "hours") + ", "
    if len(string) > 0 or minutes > 0:
        string += str(minutes) + " " + (minutes == 1 and "min" or "min")

    return string


#################################### WEATHER STATION MANAGEMENT ############################################


def readWS():
    testnetWS = checkNetwork('http://192.168.1.11')
    if int(myRobot.wsRainData) == 2:
        rainData = 'last60m_rain0_total_mm'
    elif int(myRobot.wsRainData) == 3:
        rainData = 'hour1_rain0_total_mm'
    else:
        rainData = 'last15m_rain0_total_mm'
    tempvar = int(mymower.millis) + 65000  # set nextTimeTimer (ms)
    if testnetWS:
        wsrainUrl = urllib.request.urlopen(
            'http://192.168.1.11/meteohtml.cgi?file=' + str(rainData))
        wsrain = wsrainUrl.read()
        if float(wsrain) > 0:
            send_var_message(
                'w',
                'rainWS',
                '1',
                'nextTimeTimer',
                '' + str(int(tempvar)) + '',
                '0',
                '0',
                '0',
                '0',
                '0',
            )
            if int(myRobot.wsRainData) == 1:
                print(str(float(wsrain)) + ' mm rain in last 15 minutes')
            elif int(myRobot.wsRainData) == 2:
                print(str(float(wsrain)) + ' mm rain in last 60 minutes')
            elif int(myRobot.wsRainData) == 3:
                print(float(wsrain) + ' mm rain on actual hour')
            else:
                print('WS rain data source not read from DUE yet')

        else:
            send_var_message('w', 'rainWS', '0', '0', '0', '0', '0', '0', '0',
                             '0')
            if int(myRobot.wsRainData) == 1:
                print('No rain in last 15 minutes')
            elif int(myRobot.wsRainData) == 2:
                print('No rain in last 60 minutes')
            elif int(myRobot.wsRainData) == 3:
                print('No rain in actual hour')
            else:
                print('WS rain data source not read from DUE yet')

    else:
        print('Weather station not connected..')


#################################### EMAIL ALERT MANAGEMENT ###############################################


def sendEmail():
    if (useEmailAlert):
        # point to credential file, format email:password
        with open('/home/pi/email_credentials.txt') as f:
            credentials = [x.strip().split(':', 1) for x in f]
        for email, password in credentials:
            # url to check network connection
            testnetEmail = checkNetwork('https://google.fi')
            if testnetEmail:
                print('Network check OK. Sending error email...')
                send_to_email = email  # or 'email@address.fi'

                msg = MIMEMultipart()
                msg['From'] = 'ArduMower ERROR STATE'
                msg['To'] = send_to_email
                msg['Subject'] = 'Errors and console output'

                msg.attach(
                    MIMEText(
                        'Error counters:\n' +
                        'Charger: {}\nBattery: {}\nMotor Left: {}\nMotor Right: {}\nMotor Mow: {}\nMow Sense: {}\nOdometry Left: {}\nOdometry Right: {}\nPerimeter Timeout: {}\nPerimeter Tracking: {}\nIMU Communication: {}\nIMU Calibration: {}\nIMU Tilt: {}\nRTC Communication: {}\nRTC Data: {}\nGPS Communication: {}\nGPS Data: {}\nRobot Stuck: {}\nEEPROM Data: {}\nTemp High: {}\n\n'
                        .format(mymower.ErrCharger, mymower.ErrBattery,
                                mymower.ErrMotorLeft, mymower.ErrMotorRight,
                                mymower.ErrMotorMow, mymower.ErrMowSense,
                                mymower.ErrOdoLeft, mymower.ErrOdoRight,
                                mymower.ErrPeriTout, mymower.ErrTracking,
                                mymower.ErrImuComm, mymower.ErrImuCalib,
                                mymower.ErrImuTilt, mymower.ErrRtcComm,
                                mymower.ErrRtcData, mymower.ErrGpsComm,
                                mymower.ErrGpsData, mymower.ErrStuck,
                                mymower.ErrEepromData, mymower.ErrTempHigh) +
                        'Console output:\n' + txtConsoleErr.get('1.0', 'end'),
                        'plain'))

                server = smtplib.SMTP('smtp.gmail.com', 587)
                server.starttls()
                server.login(email, password)
                text = msg.as_string()
                server.sendmail(email, send_to_email, text)
                server.quit()
                print('Error email sent!')
                # return True
                mymower.errorEmailSent = True
            else:
                print('Network connection problem. Email not sended!')
                # return False
                mymower.errorEmailSent = False


#################################### VARIABLE INITIALISATION ###############################################

direction_list = ['LEFT', 'RIGHT']
days_list = [
    'SUNDAY', 'MONDAY', 'TUESDAY', 'WEDNESDAY', 'THURSDAY', 'FRIDAY',
    'SATURDAY'
]
page_list = [
    'MAIN', 'AUTO', 'MANUAL', 'SETTING', 'CONSOLE', 'TEST', 'PLOT', 'TIMER',
    'ERROR', 'GPS', 'RFID'
]

actualRep = os.getcwd()
dateNow = time.strftime('%d/%m/%y %H:%M:%S', time.localtime())

# fen1 = tk.Tk()
'''Variable for check button '''

firstFixFlag = False
firstFixDate = 0

# fen1.title('PiArduMower')
# fen1.geometry('800x480')


class datetime:
    def __init__(self):
        datetime.hour = 12
        datetime.minute = 0
        datetime.dayOfWeek = 0
        datetime.day = 1
        datetime.month = 1
        datetime.year = 2017


class mower:
    # char* mowPatternNames[] = {'RAND', 'LANE', 'WIRE'};
    def __init__(self):
        mower.millis = 0
        mower.status = 0
        mower.state = 0
        mower.odox = 0
        mower.odoy = 0
        mower.prevYaw = 0
        mower.batVoltage = 0
        mower.yaw = 0
        mower.pitch = 0
        mower.roll = 0
        mower.version = 'Unknow'
        mower.statsOverride = 0
        mower.statsMowTimeMinutesTrip = 0
        mower.statsMowTimeHoursTotal = 0
        mower.statsBatteryChargingCounterTotal = 0
        mower.statsBatteryChargingCapacityTrip = 0
        mower.statsBatteryChargingCapacityTotal = 0
        mower.statsBatteryChargingCapacityAverage = 0
        mower.motorLeftSenseCurrent = 0
        mower.motorRightSenseCurrent = 0
        mower.motorLeftPWMCurr = 0
        mower.motorRightPWMCurr = 0
        mower.secondMowMotor = False
        mower.motor1MowSense = 0
        mower.motor2MowSense = 0
        mower.motorMowPWMCurr = 0
        mower.mowPatternCurr = 0
        mower.Dht22Temp = 0
        mower.raspiTemp = 0
        mower.Dht22Humid = 0
        mower.rollDir = 0
        mower.laneInUse = 0
        mower.YawActual = 0
        mower.YawCible = 0
        mower.loopsPerSecond = 0
        mower.useJoystick = False
        mower.lastRfidFind = 0
        mower.newtagToDo = 'FR0'
        mower.newtagRotAngle1 = 90
        mower.newtagRotAngle2 = 90
        mower.newtagSpeed = 120
        mower.newtagDistance1 = 0
        mower.newtagDistance2 = 0
        mower.laserSensor1 = 9000
        mower.laserSensor2 = 9000
        mower.laserSensor3 = 9000
        mower.laserSensor4 = 9000
        mower.laserSensor5 = 9000
        mower.laserSensor6 = 9000
        mower.rainDetect = False
        mower.areaInMowing = 1
        mower.areaToGo = 1
        mower.speedIsReduce = False
        mower.sigArea2Off = True
        mower.timeToResetSpeed = 0
        mower.timeToStartArea2Signal = 0
        mower.focusOnPage = 0
        mower.dueSerialReceived = ''
        mower.autoRecordBatChargeOn = False
        # WS
        mower.rainWS = False
        mower.wsrainLast = 0.1
        mower.timeToReadWS = time.time() + 30
        mower.timeToReadRPiTemp = time.time() + 15
        # Errors
        mower.ErrCharger = 0
        mower.ErrBattery = 0
        mower.ErrMotorLeft = 0
        mower.ErrMotorRight = 0
        mower.ErrMotorMow = 0
        mower.ErrMowSense = 0
        mower.ErrOdoLeft = 0
        mower.ErrOdoRight = 0
        mower.ErrPeriTout = 0
        mower.ErrTracking = 0
        mower.ErrImuComm = 0
        mower.ErrImuCalib = 0
        mower.ErrImuTilt = 0
        mower.ErrRtcComm = 0
        mower.ErrRtcData = 0
        mower.ErrGpsComm = 0
        mower.ErrGpsData = 0
        mower.ErrStuck = 0
        mower.ErrEepromData = 0
        mower.ErrTempHigh = 0
        mower.errorEmailSent = False
        mower.errorResetDone = True
        mower.resetChargeReadings = True
        # MQTT
        mower.mqtt_message_id = 0
        mower.callback_id = 0
        mower.timeToSendMqttState = time.time() + 150
        mower.timeToReconnectMqtt = time.time() + 120
        mower.lastMqttBatteryValue = 0
        #mower.timeToSendMqttMowPattern=time.time()+200


mymower = mower()
myRobot = robot()
myDate = datetime()

#################################### MAIN LOOP ###############################################


def checkSerial():  # the main loop is that

    while True:
        if DueConnectedOnPi:
            # try:
            mymower.dueSerialReceived = Due_Serial.readline()
            if str(mymower.dueSerialReceived) != "b''":
                mymower.dueSerialReceived = str(mymower.dueSerialReceived,
                                                'utf8')
                if mymower.dueSerialReceived[:1] != '$':  #it is console message because the first digit is not $
                    if (len(mymower.dueSerialReceived)) > 2:
                        print(mymower.dueSerialReceived)

                else:  # here a nmea message
                    # print(mymower.dueSerialReceived)
                    message = pynmea2.parse(mymower.dueSerialReceived)
                    decode_message(message)

                    # try:
                    #     message = pynmea2.parse(mymower.dueSerialReceived)
                    #     decode_message(message)
                    # except :
                    # #    print('INCOMMING MESSAGE ERROR FROM DUE --> ' + str(mymower.dueSerialReceived))
                    #     consoleInsertText('INCOMMING MESSAGE ERROR FROM DUE' + '\n')
                    #     consoleInsertText(str(mymower.dueSerialReceived) + '\n')

            # except:
            #     try:
            #         print('Due serial connection fail, trying to reset')
            #         Due_Serial.close()
            #         time.sleep(5)
            #         Due_Serial.open()
            #     except:
            #         print('Serial connection reset failed, shutdown 60s')
            #         time.sleep(60)
            #         fen1.destroy()
            #         time.sleep(1)
            #         print('Fen1 destroy')
            #         time.sleep(1)
            #         sys.exit('Impossible to continue')

        if (mower.speedIsReduce) & (time.time() > mower.timeToResetSpeed):
            mower.speedIsReduce = False
            send_var_message('w', 'MaxSpeedperiPwm',
                             '' + str(myRobot.MaxSpeedperiPwm) + '', '0', '0',
                             '0', '0', '0', '0', '0')

        if UseWeatherStation:
            if time.time() > mower.timeToReadWS:
                mower.timeToReadWS = time.time() + 60  # 60s delay
                readWS()

        if time.time() > mower.timeToReadRPiTemp:
            mower.timeToReadRPiTemp = time.time() + 30  # 30s delay
            readRasPiTemp()

        # if useDebugConsole:
        #     txtRecu.delete('5000.0', tk.END)  # keep only  lines
        #     txtSend.delete('5000.0', tk.END)  # keep only  lines
        # txtConsoleRecu.delete('2500.0', tk.END)  # keep only  lines
        # txtConsoleErr.delete('100.0', tk.END)  # keep lines

        if ((useMqtt) & (Mqqt_client.connected_flag == False) &
            (time.time() > mymower.timeToReconnectMqtt)):
            # if (Mqqt_client.connected_flag):
            #     if (time.time() > mymower.timeToSendMqttIdle):
            #         sendMqtt('Mower/Idle', str(mymower.loopsPerSecond))
            #         mymower.timeToSendMqttIdle = time.time() + 5

            # else:
            # if (time.time() > mymower.timeToReconnectMqtt):
            print('MQTT not connected retry each 2 minutes' + '\n')
            Mqqt_Connection()
            mymower.timeToReconnectMqtt = time.time() + 120

    # fen1.after(20, checkSerial)  # here is the main loop each 20ms


#################################### END OF MAINLOOP ###############################################


def decode_message(message):  # decode the nmea message
    # KeyboardFocusManager.getCurrentKeyboardFocusManager().getActiveWindow()
    if useDebugConsole:
        txtRecu.insert('1.0', str(message) + '\n')

    # receive a command from the DUE (need to do something
    if message.sentence_type == 'CMD':
        if message.actuatorname == 'RestartPi':
            if (useMqtt):
                print('Close Mqtt Connection' + '\n')
                Mqqt_DisConnection()
            print('All Console Data are saved' + '\n')
            print('The GPS Record is stopped' + '\n')
            print('PI Restart into 5 Seconds' + '\n')
            print('Start to save all Console Data' + '\n')
            # ButtonSaveReceived_click()  #save the console txt
            time.sleep(1)
            subprocess.Popen('/home/pi/Documents/PiArdumower/Restart.py')
            # fen1.destroy()
            time.sleep(1)
            # print('Fen1 is destroy')
            sys.exit('Restart ordered by Arduino Due')
        if message.actuatorname == 'PowerOffPi':
            mymower.focusOnPage = 4
            print('Start to save all Console Data' + '\n')
            # ButtonSaveReceived_click()  # save the console txt
            print('All Console Data are saved' + '\n')
            print('All Console Data are saved')
            # txtConsoleRecu.insert('1.0', 'Start to stop the GPS Record')
            # mygpsRecord.stop()
            # print('The GPS Record is stopped' + '\n')
            # print('The GPS Record is stopped')
            print('PI start Shutdown into 5 Seconds' + '\n')
            time.sleep(1)
            print(
                'Start subprocess /home/pi/Documents/PiArdumower/PowerOff.py')
            subprocess.Popen('/home/pi/Documents/PiArdumower/PowerOff.py')
            # fen1.destroy()
            time.sleep(1)
            # print('Fen1 is destroy')
            sys.exit('PowerOFF ordered by Arduino Due')

    if message.sentence_type == 'BYL':  # to refresh the ByLane setting page
        mymower.millis = int(message.millis)
        mymower.rollDir = message.rollDir
        mymower.laneInUse = message.laneInUse
        mymower.YawActual = message.YawActual
        mymower.YawCible = message.YawCible

    if message.sentence_type == 'MOT':  # to refresh the plot page of motor wheel
        mymower.millis = int(message.millis)
        mymower.motorLeftSenseCurrent = message.motorLeftSenseCurrent
        mymower.motorRightSenseCurrent = message.motorRightSenseCurrent
        mymower.motorLeftPWMCurr = message.motorLeftPWMCurr
        mymower.motorRightPWMCurr = message.motorRightPWMCurr
        mymower.batVoltage = message.batVoltage

    if message.sentence_type == 'MOW':  # to refresh the plot page of motor mow
        mymower.millis = int(message.millis)
        mymower.motor1MowSense = message.motor1MowSense
        mymower.motor2MowSense = message.motor2MowSense
        mymower.motorMowPWMCurr = message.motorMowPWMCurr
        mymower.batVoltage = message.batVoltage

    if message.sentence_type == 'BAT':  # to refresh the plot page of battery
        mymower.millis = int(message.millis)
        mymower.chgVoltage = message.chgVoltage
        mymower.chgSense = message.chgSense
        mymower.batVoltage = message.batVoltage
        mymower.resetChargeReadings = False

    if message.sentence_type == 'PER':  # to refresh the plot page of perimeter
        mymower.millis = int(message.millis)
        mymower.perimeterMag = message.perimeterMag
        mymower.perimeterMagRight = message.perimeterMagRight
        mymower.areaInMowing = int(message.areaInMowing)

    if message.sentence_type == 'STU':  # message for status info send on change only
        mymower.status = int(message.status)
        if (useMqtt and Mqqt_client.connected_flag):
            sendMqtt("Mower/Status", str(myRobot.statusNames[mymower.status]))

        if myRobot.statusNames[mymower.status] == 'TRACK_TO_START':
            mymower.areaInMowing = int(message.val1)
            mymower.areaToGo = int(message.val2)

    if message.sentence_type == 'RFI':  # message for status info send on change only
        mymower.status = int(message.status)
        mymower.lastRfidFind = message.rfidTagFind
        find_rfid_tag()

    if message.sentence_type == 'STA':  # permanent message for state info

        mymower.millis = int(message.millis)
        mymower.odox = message.odox
        mymower.odoy = message.odoy
        mymower.prevYaw = message.prevYaw

        if useMqtt:
            if ((mymower.state != int(message.state)) |
                (time.time() > mymower.timeToSendMqttState)):
                mymower.timeToSendMqttState = time.time() + 300
                mymower.state = int(message.state)
                sendMqtt("Mower/State", str(myRobot.stateNames[mymower.state]))
            if (mymower.batVoltage != float(message.batVoltage)):
                mymower.batVoltage = float(message.batVoltage)
                ecart = mower.lastMqttBatteryValue - float(message.batVoltage)
                # only send Mqtt if 0.2 volt dif
                if (abs(ecart) >= 0.2):
                    sendMqtt('Mower/Battery', message.batVoltage)
                    mower.lastMqttBatteryValue = float(message.batVoltage)
            if (mymower.Dht22Temp != message.Dht22Temp):
                mymower.Dht22Temp = message.Dht22Temp
                sendMqtt("Mower/Temp", str(mymower.Dht22Temp))
        else:
            mymower.batVoltage = float(message.batVoltage)
            mymower.Dht22Temp = message.Dht22Temp
            mymower.state = int(message.state)

        mymower.yaw = message.yaw
        mymower.pitch = message.pitch
        mymower.roll = message.roll

        mymower.loopsPerSecond = message.loopsPerSecond

        if ((myRobot.stateNames[mymower.state] != 'CHARG') &
            (mymower.resetChargeReadings == True)):
            mymower.resetChargeReadings = False

        if ((myRobot.stateNames[mymower.state] != 'ERR') &
            (mymower.errorResetDone == False)):
            mymower.errorEmailSent = False
            mymower.errorResetDone = True

    if message.sentence_type == 'ERR':  # Errors message
        mymower.millis = message.millis
        mymower.ErrCharger = message.errCharger
        mymower.ErrBattery = message.errBattery
        mymower.ErrMotorLeft = message.errMotorLeft
        mymower.ErrMotorRight = message.errMotorRight
        mymower.ErrMotorMow = message.errMotorMow
        mymower.ErrMowSense = message.errMowSense
        mymower.ErrOdoLeft = message.errOdoLeft
        mymower.ErrOdoRight = message.errOdoRight
        mymower.ErrPeriTout = message.errPeriTout
        mymower.ErrTracking = message.errTracking
        mymower.ErrImuComm = message.errImuComm
        mymower.ErrImuCalib = message.errImuCalib
        mymower.ErrImuTilt = message.errImuTilt
        mymower.ErrRtcComm = message.errRtcComm
        mymower.ErrRtcData = message.errRtcData
        mymower.ErrGpsComm = message.errGpsComm
        mymower.ErrGpsData = message.errGpsData
        mymower.ErrStuck = message.errStuck
        mymower.ErrEepromData = message.errEepromData
        mymower.ErrTempHigh = message.errTempHigh
        mymower.errorResetDone = False

        if (myRobot.stateNames[mymower.state] == 'ERR') & (
                mymower.errorEmailSent == False):
            # ButtonSaveReceived_click()  # save the console txt
            checkMailSent = sendEmail()
            # if checkMailSent:
            #     mymower.errorEmailSent = True
            # else:
            #     mymower.errorEmailSent = False

    # to fill the setting page All or name of the needed page
    if (message.sentence_type == 'RET'):
        if message.setting_page == 'Time':
            myDate.hour = message.val1
            myDate.minute = message.val2
            myDate.dayOfWeek = message.val3
            myDate.day = message.val4
            myDate.month = message.val5
            myDate.year = message.val6

            cmdline = "sudo date -s " + "'" + myDate.year + "-"
            cmdline = cmdline + myDate.month + "-" + myDate.day + " "
            cmdline = cmdline + myDate.hour + ":" + myDate.minute + ":"
            cmdline = cmdline + "0" + "'"

            if myOS == 'Linux':
                print('Set the new time and date to PI')
                print(cmdline)
                os.system(cmdline)

            dateNow = time.strftime('%d/%m/%y %H:%M:%S', time.localtime())
            print(dateNow)

        if message.setting_page == 'All':
            if message.pageNr == '1':
                myRobot.developerActive = message.val1
                myRobot.motorAccel = message.val2
                myRobot.motorSpeedMaxRpm = message.val3
                myRobot.motorSpeedMaxPwm = message.val4
                myRobot.motorPowerMax = message.val5
                myRobot.motorSenseRightScale = message.val6
                myRobot.motorSenseLeftScale = message.val7
                myRobot.motorRollDegMax = message.val8
                myRobot.motorRollDegMin = message.val9
                myRobot.DistPeriOutRev = message.val10
            if message.pageNr == '2':
                myRobot.motorPowerIgnoreTime = message.val1
                myRobot.motorForwTimeMax = message.val2
                myRobot.motorMowSpeedMaxPwm = message.val3
                myRobot.motorMowPowerMax = message.val4
                myRobot.motorMowRPMSet = message.val5
                myRobot.motor1MowSenseScale = message.val6
                myRobot.motorLeftPID_Kp = message.val7
                myRobot.motorLeftPID_Ki = message.val8
                myRobot.motorLeftPID_Kd = message.val9
                myRobot.motorMowPID_Kp = message.val10
            if message.pageNr == '3':
                myRobot.motorMowPID_Ki = message.val1
                myRobot.motorMowPID_Kd = message.val2
                myRobot.motorBiDirSpeedRatio1 = message.val3
                myRobot.motorBiDirSpeedRatio2 = message.val4
                myRobot.motorLeftSwapDir = message.val5
                myRobot.motorRightSwapDir = message.val6
                myRobot.bumperUse = message.val7
                myRobot.sonarUse = message.val8
                myRobot.sonarCenterUse = message.val9
                myRobot.sonarLeftUse = message.val10
            if message.pageNr == '4':
                myRobot.sonarRightUse = message.val1
                myRobot.sonarTriggerBelow = message.val2
                myRobot.perimeterUse = message.val3
                myRobot.perimeter_timedOutIfBelowSmag = message.val4
                # myRobot.perimeterTriggerTimeout = message.val5
                myRobot.perimeterTriggerMinSmag = message.val5
                myRobot.perimeterOutRollTimeMax = message.val6
                myRobot.perimeterOutRollTimeMin = message.val7
                myRobot.perimeterOutRevTime = message.val8
                myRobot.perimeterTrackRollTime = message.val9
                myRobot.perimeterTrackRevTime = message.val10
            if message.pageNr == '5':
                myRobot.perimeterPID_Kp = message.val1
                myRobot.perimeterPID_Ki = message.val2
                myRobot.perimeterPID_Kd = message.val3
                myRobot.perimeter_signalCodeNo = message.val4
                myRobot.perimeter_swapCoilPolarityLeft = message.val5
                myRobot.perimeter_timeOutSecIfNotInside = message.val6
                myRobot.trakBlockInnerWheel = message.val7
                myRobot.lawnSensorUse = message.val8
                myRobot.imuUse = message.val9
                myRobot.stopMotorDuringCalib = message.val10
            if message.pageNr == '6':
                myRobot.imuDirPID_Kp = message.val1
                myRobot.imuDirPID_Ki = message.val2
                myRobot.imuDirPID_Kd = message.val3
                myRobot.imuRollPID_Kp = message.val4
                myRobot.imuRollPID_Ki = message.val5
                myRobot.imuRollPID_Kd = message.val6
                myRobot.remoteUse = message.val7
                myRobot.batMonitor = message.val8
                myRobot.batGoHomeIfBelow = message.val9
                myRobot.batSwitchOffIfBelow = message.val10
            if message.pageNr == '7':
                myRobot.batSwitchOffIfIdle = message.val1
                myRobot.batFactor = message.val2
                myRobot.batChgFactor = message.val3
                myRobot.chgSenseZero = message.val4
                myRobot.batSenseFactor = message.val5
                myRobot.batFullCurrent = message.val6
                myRobot.startChargingIfBelow = message.val7
                myRobot.stationRevDist = message.val8
                myRobot.stationRollAngle = message.val9
                myRobot.stationForwDist = message.val10
            if message.pageNr == '8':
                myRobot.stationCheckDist = message.val1
                myRobot.odometryUse = message.val2
                myRobot.odometryTicksPerRevolution = message.val3
                myRobot.odometryTicksPerCm = message.val4
                myRobot.odometryWheelBaseCm = message.val5
                myRobot.autoResetActive = message.val6
                myRobot.odometryRightSwapDir = message.val7
                myRobot.twoWayOdometrySensorUse = message.val8
                myRobot.buttonUse = message.val9
                myRobot.userSwitch1 = message.val10
            if message.pageNr == '9':
                myRobot.userSwitch2 = message.val1
                myRobot.userSwitch3 = message.val2
                myRobot.timerUse = message.val3
                myRobot.rainUse = message.val4
                myRobot.gpsUse = message.val5
                myRobot.stuckIfGpsSpeedBelow = message.val6
                myRobot.gpsSpeedIgnoreTime = message.val7
                myRobot.dropUse = message.val8
                myRobot.statsOverride = message.val9
                myRobot.bluetoothUse = message.val10
            if message.pageNr == '10':
                myRobot.esp8266Use = message.val1
                myRobot.esp8266ConfigString = message.val2
                myRobot.tiltUse = message.val3
                myRobot.trackingPerimeterTransitionTimeOut = message.val4
                myRobot.motorMowForceOff = message.val5
                myRobot.MaxSpeedperiPwm = message.val6
                myRobot.RollTimeFor45Deg = message.val7
                myRobot.DistPeriObstacleAvoid = message.val8
                myRobot.circleTimeForObstacle = message.val9
                myRobot.DistPeriOutRev = message.val10
            if message.pageNr == '11':
                myRobot.motorRightOffsetFwd = message.val1
                myRobot.motorRightOffsetRev = message.val2
                myRobot.perimeterMagMaxValue = message.val3
                myRobot.SpeedOdoMin = message.val4
                myRobot.SpeedOdoMax = message.val5
                myRobot.yawSet1 = message.val6
                myRobot.yawSet2 = message.val7
                myRobot.yawSet3 = message.val8
                myRobot.yawOppositeLane1RollRight = message.val9
                myRobot.yawOppositeLane2RollRight = message.val10
            if message.pageNr == '12':
                myRobot.yawOppositeLane3RollRight = message.val1
                myRobot.yawOppositeLane1RollLeft = message.val2
                myRobot.yawOppositeLane2RollLeft = message.val3
                myRobot.yawOppositeLane3RollLeft = message.val4
                myRobot.DistBetweenLane = message.val5
                myRobot.maxLenghtByLane = message.val6
                myRobot.perimeter_swapCoilPolarityRight = message.val7
                myRobot.perimeter_read2Coil = message.val8
                myRobot.maxDriftPerSecond = message.val9
                myRobot.delayBetweenTwoDmpAutocalib = message.val10
            if message.pageNr == '13':
                myRobot.maxDurationDmpAutocalib = message.val1
                myRobot.mowPatternDurationMax = message.val2
                myRobot.DistPeriOutStop = message.val3
                myRobot.DHT22Use = message.val4
                myRobot.RaspberryPIUse = message.val5
                myRobot.motor2MowSenseScale = message.val6
                myRobot.secondMowMotor = message.val7
                myRobot.rainReadDelay = message.val8
                myRobot.maxTemperature = message.val9
                myRobot.wsRainData = message.val10
            if message.pageNr == '14':
                myRobot.useBumperDock = message.val1
                myRobot.dockingSpeed = message.val2
                myRobot.motorLeftSpeedDivider = message.val3
                myRobot.raspiTempUse = message.val4
                myRobot.raspiTempMax = message.val5

        if message.setting_page == 'Motor':
            if message.pageNr == '1':
                myRobot.motorPowerMax = message.val1
                myRobot.motorSpeedMaxRpm = message.val2
                myRobot.motorSpeedMaxPwm = message.val3
                myRobot.motorAccel = message.val4
                myRobot.motorPowerIgnoreTime = message.val5
                myRobot.motorRollDegMax = message.val6
                myRobot.motorRollDegMin = message.val7
                myRobot.DistPeriOutRev = message.val8
                myRobot.DistPeriOutStop = message.val9
                myRobot.motorLeftPID_Kp = message.val10

            if message.pageNr == '2':
                myRobot.motorLeftPID_Ki = message.val1
                myRobot.motorLeftPID_Kd = message.val2
                myRobot.motorLeftSwapDir = message.val3
                myRobot.motorRightSwapDir = message.val4
                myRobot.motorRightOffsetFwd = message.val5
                myRobot.motorRightOffsetRev = message.val6
                myRobot.SpeedOdoMin = message.val7
                myRobot.SpeedOdoMax = message.val8
                myRobot.motorSenseLeftScale = message.val9
                myRobot.motorSenseRightScale = message.val10

    if message.sentence_type == 'CFG':
        # text1.config(text= message.debug)
        print(message.debug)

    if message.sentence_type == 'DEB':
        # text1.config(text= message.debug)
        print(message.debug)


# def ButtonSaveReceived_click():
#     fileName = ('/home/pi/Documents/PiArdumower/log/' +
#                 time.strftime('%Y%m%d%H%M') + '_Received.txt')
#     with open(fileName, 'w') as f:
#         f.write(txtRecu.get('1.0', 'end'))
#     fileName = ('/home/pi/Documents/PiArdumower/log/' +
#                 time.strftime('%Y%m%d%H%M') + '_Send.txt')
#     with open(fileName, 'w') as f:
#         f.write(txtSend.get('1.0', 'end'))
#     fileName = ('/home/pi/Documents/PiArdumower/log/' +
#                 time.strftime('%Y%m%d%H%M') + '_Console.txt')
#     with open(fileName, 'w') as f:
#         f.write(txtConsoleRecu.get('1.0', 'end'))
#     print('All Console file are saved' + '\n')


def button_stop_all_click():
    send_pfo_message(
        'ro',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def ButtonInfo_click():
    send_req_message(
        'INF',
        '2',
        '0',
        '1',
        '0',
        '0',
        '0',
    )


def ButtonGyroCal_click():
    send_pfo_message(
        'g18',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def ButtonCompasCal_click():
    send_pfo_message(
        'g19',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def ButtonSendSettingToEeprom_click():
    send_pfo_message(
        'sz',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


# def ButtonSendSettingDateTimeToDue_click():
#     Send_reqSetting_message('Time','w','1',''+str(tk_date_hour.get())+\
#                             '',''+str(tk_date_minute.get())+\
#                             '',''+str(tk_date_dayOfWeek.get())+\
#                             '',''+str(tk_date_day.get())+\
#                             '',''+str(tk_date_month.get())+\
#                             '',''+str(tk_date_year.get())+\
#                             '',''+str(0)+\
#                             '',''+str(0)+\
#                             '',''+str(0)+\
#                             '',''+str(0)+'',)


def read_all_setting():
    Send_reqSetting_message('All', 'r', '0', '0', '0', '0', '0', '0', '0', '0',
                            '0', '0', '0')


def read_time_setting():
    Send_reqSetting_message('Time', 'r', '0', '0', '0', '0', '0', '0', '0',
                            '0', '0', '0', '0')


def send_req_message(val1, val2, val3, val4, val5, val6, val7):
    message = pynmea2.VAR('RM', 'REQ',
                          (val1, val2, val3, val4, val5, val6, val7))
    message = str(message)
    rpiCheckSum = (message[-2:])
    message = message + '\r' + '\n'
    send_serial_message(message)


def send_var_message(val1, val2, val3, val4, val5, val6, val7, val8, val9,
                     val10):
    message = pynmea2.VAR(
        'RM', 'VAR',
        (val1, val2, val3, val4, val5, val6, val7, val8, val9, val10))
    message = str(message)
    rpiCheckSum = (message[-2:])
    message = message + '\r' + '\n'
    send_serial_message(message)


def send_cmd_message(val1, val2, val3, val4, val5):
    message = pynmea2.CMD('RM', 'CMD', (val1, val2, val3, val4, val5))
    message = str(message)
    rpiCheckSum = (message[-2:])
    message = message + '\r' + '\n'
    send_serial_message(message)


def send_pfo_message(val1, val2, val3, val4, val5, val6, val7):
    message = pynmea2.PFO('RM', 'PFO',
                          (val1, val2, val3, val4, val5, val6, val7))
    message = str(message)
    rpiCheckSum = (message[-2:])
    message = message + '\r' + '\n'
    send_serial_message(message)


def Send_reqSetting_message(val1, val2, val3, val4, val5, val6, val7, val8,
                            val9, val10, val11, val12, val13):
    message = pynmea2.SET('RM', 'SET',
                          (val1, val2, val3, val4, val5, val6, val7, val8,
                           val9, val10, val11, val12, val13))
    message = str(message)
    rpiCheckSum = (message[-2:])
    message = message + '\r' + '\n'
    send_serial_message(message)


def send_serial_message(message1):
    try:
        if DueConnectedOnPi:
            # checkSerial()
            Due_Serial.flushOutput()
            Due_Serial.write(bytes(message1, 'utf-8'))
            # print('Send Message :' , message1)
            if useDebugConsole:
                txtSend.insert('1.0', message1)
    except:
        print('ERREUR while transfert')
        time.sleep(2)


''' ------------------- connecting the the PCB1.3 arduino due ------------ '''
try:
    if DueConnectedOnPi:
        if myOS == 'Linux':
            Due_Serial = serial.Serial('/dev/ttyACM_DUE', 115200, timeout=0)
        # if os.path.exists('/dev/ttyACM0') == True:
        #     Due_Serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
        #     print('Find Serial on ttyACM0')
        # if os.path.exists('/dev/ttyACM1') == True:
        #     Due_Serial = serial.Serial('/dev/ttyACM1', 115200, timeout=0)
        #     print('Find Serial on ttyACM1')
        else:
            Due_Serial = serial.Serial('COM9', 115200, timeout=0)
        byteResponse = Due_Serial.readline()
        print(str(byteResponse))

except:
    print(' ')
    print('************************************')
    print('ERREUR DE CONNECTION WITH PCB1.3 DUE')
    print('************************************')
    print(' ')
    time.sleep(1)
    #sys.exit('Impossible to continue')


def ButtonReboot_click():
    print("reboot")
    send_pfo_message(
        'h04',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def ButtonRollDir_click():
    send_pfo_message(
        'w20',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def button_home_click():
    tempVar = int(mymower.millis) + 3600000
    send_var_message('w', 'nextTimeTimer', '' + str(int(tempVar)) + '', '0',
                     '0', '0', '0', '0', '0', '0')
    send_var_message('w', 'statusCurr', '3', '0', '0', '0', '0', '0', '0',
                     '0')  # set the status to back to station
    send_pfo_message(
        'rh',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def button_track_click():
    send_pfo_message(
        'rk',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def buttonStartMow_click():
    send_var_message('w', 'mowPatternCurr',
                     '' + str(mymower.mowPatternCurr) + '', '0', '0', '0', '0',
                     '0', '0', '0')
    send_pfo_message(
        'ra',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def buttonBlade_stop_click():
    # set the state to Manual before stop the blade
    send_var_message('w', 'stateCurr', '17', '0', '0', '0', '0', '0', '0', '0')
    send_cmd_message('mowmotor', '0', '0', '0', '0')


def buttonBlade_start_click():
    # set the state to Manual before start the blade
    send_var_message('w', 'stateCurr', '17', '0', '0', '0', '0', '0', '0', '0')
    send_cmd_message('mowmotor', '1', '0', '0', '4')


''' The Console page  ************************************'''


def ButtonListVar_click():
    send_pfo_message(
        'h02',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


def ButtonConsoleMode_click():
    send_pfo_message(
        'h03',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


# ConsolePage = tk.Frame(fen1)
# ConsolePage.place(x=0, y=0, height=400, width=800)

# txtRecu = tk.Text(ConsolePage)
# ScrollTxtRecu = tk.Scrollbar(txtRecu)
# ScrollTxtRecu.pack(side=tk.RIGHT, fill=tk.Y)
# txtRecu.pack(side=tk.LEFT, fill=tk.Y)
# ScrollTxtRecu.config(command=txtRecu.yview)
# txtRecu.config(yscrollcommand=ScrollTxtRecu.set)
# txtRecu.place(x=0, y=300, anchor='nw', width=480, height=100)

# txtSend = tk.Text(ConsolePage)
# ScrollTxtSend = tk.Scrollbar(txtSend)
# ScrollTxtSend.pack(side=tk.RIGHT, fill=tk.Y)
# txtSend.pack(side=tk.LEFT, fill=tk.Y)
# ScrollTxtSend.config(command=txtSend.yview)
# txtSend.config(yscrollcommand=ScrollTxtSend.set)
# txtSend.place(x=490, y=300, anchor='nw', width=300, height=100)

# txtConsoleRecu = tk.Text(ConsolePage)
# ScrolltxtConsoleRecu = tk.Scrollbar(txtConsoleRecu)
# ScrolltxtConsoleRecu.pack(side=tk.RIGHT, fill=tk.Y)
# txtConsoleRecu.pack(side=tk.LEFT, fill=tk.Y)
# ScrolltxtConsoleRecu.config(command=txtConsoleRecu.yview)
# txtConsoleRecu.config(yscrollcommand=ScrolltxtConsoleRecu.set)
# txtConsoleRecu.place(x=0, y=5, anchor='nw', width=800, height=290)

# # Console window
# txtConsoleErr = tk.Text(ConsolePage)
# ScrolltxtConsoleErr = tk.Scrollbar(txtConsoleErr)
# ScrolltxtConsoleErr.pack(side=tk.RIGHT, fill=tk.Y)
# txtConsoleErr.pack(side=tk.LEFT, fill=tk.Y)
# ScrolltxtConsoleErr.config(command=txtConsoleErr.yview)
# txtConsoleErr.config(yscrollcommand=ScrolltxtConsoleErr.set)
# txtConsoleErr.place(x=2 * 400,
#                     y=10,
#                     anchor='nw',
#                     width=800 - (2 * 400),
#                     height=370)
''' THE RFID PAGE ***************************************************'''

# Read the file and create the list
with open('/home/pi/Documents/PiArdumower/tag_list.bin', 'rb') as fp:
    # rfid_list=[['01254456700','FR0',60,0,0,0,0,0],['01254456711','FR1',61,1,1,0,0,0]]
    rfid_list = pickle.load(fp)
    print('RFID file loaded OK')
    # print(rfid_list)

# def handler(event):
#     current = combobox.current()
#     if current != -1:
#         print(myRobot.stateNames[current])

# combobox = ttk.Combobox(RfidPage,values=myRobot.stateNames, height=60, width=140)
# combobox.bind('<<ComboboxSelected>>', handler)
# combobox.current(0)
# combobox.place(x=530, y=310, height=60, width=140)
''' THE ERROR PAGE ***************************************************'''


def ButtonResetCounters_click():
    send_pfo_message('z00', '1', '2', '3', '4', '5', '6')
    time.sleep(5)


''' THE MAIN PAGE ***************************************************'''


def ButtonPowerOff_click():
    button_stop_all_click()
    send_pfo_message(
        'rt',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    )


checkSerial()

print('Read Setting from PCB1.3' + '\n')
read_all_setting()
print('Read Area In Mowing from PCB1.3' + '\n')
send_req_message(
    'PERI',
    '1000',
    '1',
    '1',
    '0',
    '0',
    '0',
)
if (useMqtt):
    print('Wait update Date/Time from internet' + '\n')
    print('Initial start MQTT after 1 minute' + '\n')
    mymower.timeToReconnectMqtt = time.time() + 60
else:
    print('Adjust PI time from PCB1.3' + '\n')
    read_time_setting()

# fen1.mainloop()