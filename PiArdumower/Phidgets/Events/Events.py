"""Copyright 2010 Phidgets Inc.
This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/
"""

__author__ = 'Adam Stelmack'
__version__ = '2.1.9'
__date__ = 'May 17 2010'

from threading import *
from ctypes import *

class AttachEventArgs:
    """Attach event data and information will be stored in this class.
    
    The data stored in this event args class is a reference to the Phidget object that triggered the event.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
    """
    def __init__(self, device):
        self.device = device

class DetachEventArgs:
    """Detach event data and information will be stored in this class.
    
    The data stored in this event args class is a reference to the Phidget object that triggered the event.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
    """
    def __init__(self, device):
        self.device = device

class ServerConnectArgs:
    """Server connect event data and information will be stored in this class.
    
    The data stored in this event args class is a reference to the Phidget, or Dictionary, or Manager object that triggered the event.
    
    Properties:
        device<object>: Reference to the Phidget, or Dictionary, or Manager object from which this event originated
    """
    def __init__(self, device):
        self.device = device

class ServerDisconnectArgs:
    """Server Disconnect event data and information will be stored in this class.
    
    The data stored in this event args class is a reference to the Phidget, or Dictionary, or Manager object that triggered the event.
    
    Properties:
        device<object>: Reference to the Phidget, or Dictionary, or Manager object from which this event originated
    """
    def __init__(self, device):
        self.device = device

class ErrorEventArgs:
    """Error event data and information will be stored in this class.
    
    The data stored in this event args class is the error description and the error code.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        description<string>:  The reference to the error description string generated by the event. 
        eCode<int>: The reference to the error code value generated by the event.
    """
    def __init__(self, device, description, eCode):
        self.device = device
        self.description = description
        self.eCode = eCode

class SensorChangeEventArgs:
    """Analog Sensor Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the analog sensor input that is changing and the sensor value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the analog sensor input index from which this event originated.
        value<int>: The reference to the value of the sensor.
    """
    def __init__(self, device, index, value):
        self.device = device
        self.index = index
        self.value = value

class SensorUpdateEventArgs:
    """Analog Sensor Update Event data and information will be stored in this class.

    Data specific to this event args class are the index of the analog sensor input that is changing and the sensor value read.

    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the analog sensor input index from which this event originated.
        value<int>: The reference to the value of the sensor.
    """
    def __init__(self, device, index, value):
        self.device = device
        self.index = index
        self.value = value

class InputChangeEventArgs:
    """Input Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the digital input that is changing and the state value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the digital input index from which this event originated.
        value<int>: The reference to the bool state value that was read to generate the event.
    """
    def __init__(self, device, index, state):
        self.device = device
        self.index = index
        self.state = state

class OutputChangeEventArgs:
    """Output Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the digital output that is changing and the state value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the index of the output.
        value<int>: The reference to the state of the output.
    """
    def __init__(self, device, index, state):
        self.device = device
        self.index = index
        self.state = state

class EncoderPositionChangeEventArgs:
    """Encoder Position Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the encoder that is changing, the position change value read,
    and the elapsed time between position changes.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>:  The reference to the encoder index from which this event originated. 
        time<int>: The reference to the elapsed time between change events.
        positionChange<int>: The reference to the position change value that was read to generate the event.
    """
    def __init__(self, device, index, time, positionChange):
        self.device = device
        self.index = index
        self.time = time
        self.positionChange = positionChange

class EncoderPositionUpdateEventArgs:
    """Encoder Position Update Event data and information will be stored in this class.

    Data specific to this event args class are the index of the encoder that is changing, the position change value read.

    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>:  The reference to the encoder index from which this event originated.
        positionChange<int>: The reference to the position change value that was read to generate the event.
    """
    def __init__(self, device, index, positionChange):
        self.device = device
        self.index = index
        self.positionChange = positionChange

class AccelerationChangeEventArgs:
    """Acceleration Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the axis that is changing and the acceleration value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the axis index from which this event originated.
        acceleration<double>: The reference to the acceleration value that was read to generate the event.
    """
    def __init__(self, device, index, acceleration):
        self.device = device
        self.index = index
        self.acceleration = acceleration

class VelocityChangeEventArgs:
    """Velocity Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the motor whose velocity is changing and the velocity value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>:  The reference to the index of the motor.
        velocity<double>: The reference to the velocity of the motor.
    """
    def __init__(self, device, index, velocity):
        self.device = device
        self.index = index
        self.velocity = velocity

class CurrentChangeEventArgs:
    """Current Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the motor whose current draw is changing and the current value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the index of the motor.
        current<double>: The reference the current of the motor.
    """
    def __init__(self, device, index, current):
        self.device = device
        self.index = index
        self.current = current

class CurrentUpdateEventArgs:
    """Current Update Event data and information will be stored in this class.

    Data specific to this event args class are the index of the motor whose current draw is changing and the current value read.

    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the index of the motor.
        current<double>: The reference the current of the motor.
    """
    def __init__(self, device, index, current):
        self.device = device
        self.index = index
        self.current = current

class PositionChangeEventArgs:
    """Servo Position Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the motor whose position is changing and the position value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the index of the motor.
        position<double>: The reference to the position of the servo motor.
    """
    def __init__(self, device, index, position):
        self.device = device
        self.index = index
        self.position = position

class SpatialDataEventArgs:
    """
    
    Properties:
        spatialData<list of objects>: list of spatialData objects
    """
    def __init__(self, device, spatialDataCollection):
        self.device = device
        self.spatialData = spatialDataCollection

class StepperPositionChangeEventArgs:
    """Stepper Position Change Event data and information will be stored in this class.
    
    Data specific to this event args class are the index of the stepper motor whose position is changing and the position value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the index of the stepper motor.
        position<double>: The reference to the position of the stepper motor.
    """
    def __init__(self, device, index, position):
        self.device = device
        self.index = index
        self.position = position

class PHChangeEventArgs:
    """PH Change Event data and information will be stored in this class.
    
    Data specific to this event args class is the PH value read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        PH<double>: The reference pH value.
    """
    def __init__(self, device, PH):
        self.device = device
        self.PH = PH

class TagEventArgs:
    """Tag Event data and information will be stored in this class.
    
    Data specific to this event args class is the Tag data that is read.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        tag<string>: The reference to the gained or lost tag.
    """
    def __init__(self, device, tag):
        self.device = device
        self.tag = tag

class TemperatureChangeEventArgs:
    """Temperature Change Event data and information will be stored in this class.
    
    Data specific to this event args class is the index of the temperature sensor and the temperature data.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        index<int>: The reference to the index of the sensor.
        temperature<double>: The reference to the temperature of the sensor.
        potential<double>: The reference to the potential of the sensor.
    """
    def __init__(self, device, index, temperature, potential):
        self.device = device
        self.index = index
        self.temperature = temperature
        self.potential = potential

class KeyChangeEventArgs:
    """Key Event data and information will be stored in this class.
    
    Data specific to this event args class is the value data and the key.
    
    Properties:
        device<object>: Reference to the Phidget object from which this event originated
        key<string>: The Key that is changing in the Dictionary.
        value<string>: The value data that is being associated with that key in the Dictionary.
    """
    def __init__(self, device, key, value, reason):
        self.device = device
        self.key = key
        self.value = value
        self.reason = reason

class IRCodeEventArgs:
    """
    """
    def __init__(self, device, code, repeat):
        self.device = device
        self.code = code
        self.repeat = repeat

class IRLearnEventArgs:
    """
    """
    def __init__(self, device, code, codeInfo):
        self.device = device
        self.code = code
        self.codeInfo = codeInfo

class IRRawDataEventArgs:
    """
    """
    def __init__(self, device, rawData):
        self.device = device
        self.rawData = rawData

class BridgeDataEventArgs:
    """

    """
    def __init__(self, device, index, value):
        self.device = device
        self.index = index
        self.value = value

class FrequencyCounterCountEventArgs:
    """

    """
    def __init__(self, device, index, time, counts):
        self.device = device
        self.index = index
        self.time = time
        self.counts = counts

class GPSPositionChangeEventArgs:
    """

    """
    def __init__(self, device, latitude, longitude, altitude):
        self.device = device
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

class GPSPositionFixStatusChangeEventArgs:
    """

    """
    def __init__(self, device, status):
        self.device = device
        self.positionFixStatus = status;

class BackEMFEventArgs:
    """

    """
    def __init__(self, device, index, voltage):
        self.device = device
        self.index = index
        self.voltage = voltage