import os

cwd = os.getcwd()


if (os.name == 'nt'):
    print('Windows Platform')
    myComPort = 'COM9'
    myFrameWidth = 800
    myFrameHeight = 430
    myBaudRate = 115200
    myOS = "Windows"
    GpsConnectedOnPi = False
    GpsIsM6n = True
    RfidConnectedOnPi = False
    NanoConnectedOnPi = False
    DueConnectedOnPi = False
    UseWeatherStation = False
    autoRecordBatCharge = False
    useDebugConsole = True

if (os.name == 'posix'):
    print('Linux Platform')
    myComPort = '/dev/ttyACM_DUE'
    myFrameWidth = 800
    myFrameHeight = 430
    myBaudRate = 115200
    myOS = "Linux"
    GpsConnectedOnPi = False
    GpsIsM6n = True
    RfidConnectedOnPi = False
    NanoConnectedOnPi = False
    DueConnectedOnPi = True
    UseWeatherStation = True
    autoRecordBatCharge = False
    useDebugConsole = True
