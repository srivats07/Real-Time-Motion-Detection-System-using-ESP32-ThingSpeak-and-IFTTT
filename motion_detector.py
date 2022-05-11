# ECE 568 - Lab Assignment - 3

import machine
from machine import Pin, PWM, Timer, I2C
import adafruit_mpu6050
import time
import urequests
import network
import socket
import json

## Global variables and Initialization

# State of the device
deviceState = False

# Wifi Credentials
wifi_ssid = 'POCO M2 Pro'
wifi_password = '24681357'
# wifi_ssid = 'NETGEAR73'
# wifi_password = 'roundlake377'

# Network Station interface
sta_if = network.WLAN(network.STA_IF)

# Adafruit MPU6050 I2C Device Address
mpu6050Address = 0x68

# Base address of Accelerometer Data in Memory
baseAddressOfAccelDataInMemory = 0x3B

# MPU6050 Accelerometer ideal values when placed on flat surface facing upwards
idealAccelX = 0
idealAccelY = 0 
idealAccelZ = 1

# Accelerometer movement threshold values
thresholdX = 0.05
thresholdY = 0.05
thresholdZ = 0.05

# Accelerometer sensor offset values
offsetValues = {}

# Previous accelerometer sensor values
previousAccelValues = {"Ax" : 0, "Ay" : 0, "Az" : 0}

# ThingSpeak credentials
thingspeakReadKey = "XDNYYPRA5G4IZY0D"
thingspeakApiHost = "api.thingspeak.com"
thingspeakApiPort = 80
thingspeakChannelId = 1726436

# Initialize Pin 12 and Pin 13 as OUT for the external LEDs
greenLed = machine.Pin(12, Pin.OUT)
greenLed.value(0)
redLed = machine.Pin(13, Pin.OUT)
redLed.value(0)

# Webhook credentials
webhookEventName = "Motion_detect"
webhookKey = "dHf5S-Ys1kKufNt9jcGtbs"

## Subroutines

# Function connects ESP32 board to Wireless Network.
def ConnectToWifi(ssid, password):  
    sta_if.active(True)
    print(sta_if.isconnected())
    if sta_if.isconnected():
        PrintWirelessNetworkDetails(sta_if)
        return None
    print('Trying to connect to %s...' % ssid)
    sta_if.connect(ssid, password)
    for retry in range(100):
        connected = sta_if.isconnected()
        if connected:
            break
        time.sleep(0.1)
        print('.', end='')
    if connected:
        PrintWirelessNetworkDetails(sta_if)
    else:
        print("Error connecting...Check credentials and reboot the system")

# Function displays Wireless network connection details.
def PrintWirelessNetworkDetails(sta_if):
    print("\n")
    print("Connected to " + sta_if.config('essid'))
    print("IP Address:", sta_if.ifconfig()[0])

# Print address of I2C Devices connected
def PrintI2CDeviceAddress(deviceAddresses):
    if deviceAddresses:
        print("Printing the address of the connected peripheral devices")
        for i in deviceAddresses:
            print(f"Address: {hex(i)}")
    else:
        print("No I2C Peripheral devices found. Check device connection")

# Get Mean Accelerometer values
def GetMeanAccelerometerValues():
    numberOfSamples = 10
    meanValue = {"Ax" : 0, "Ay" : 0, "Az" : 0}
    sumValue = {"Ax" : 0, "Ay" : 0, "Az" : 0}
    for i in range(1, numberOfSamples):
        accelData = GetAccelerometerValues()
        sumValue['Ax']  = sumValue['Ax'] + accelData['Ax']
        sumValue['Ay']  = sumValue['Ay'] + accelData['Ay']
        sumValue['Az']  = sumValue['Az'] + accelData['Az']
    meanValue['Ax'] = sumValue['Ax']/numberOfSamples
    meanValue['Ay'] = sumValue['Ay']/numberOfSamples
    meanValue['Az'] = sumValue['Az']/numberOfSamples
    return meanValue

# Calculate Accelerometer Offset Values
def CalculateAccelerometerOffsetValues():
    meanAccelValues = GetAccelerometerValues()
    
    # Calculate offset values from obtained values
    offsetValues["Ax"] = idealAccelX - meanAccelValues["Ax"]
    offsetValues["Ay"] = idealAccelY - meanAccelValues["Ay"]
    offsetValues["Az"] = idealAccelZ - meanAccelValues["Az"]
    
    return offsetValues
    
# Check if Adafruit MPU6050 sensor is connected to I2C
def CheckMpu6050DeviceIsConnected(deviceAddresses):
    if mpu6050Address in deviceAddresses:
        return true
    else: return false
        
# Get raw accelerometer values from the memory
def GetAccelerometerValues():
    # Read 6 bytes of accelerometer data from the memory location
    rawData = i2c.readfrom_mem(mpu6050Address, baseAddressOfAccelDataInMemory, 6)

    # Calculate offset corrected accelerometer values
    accelX = ConvertBytesToInt(rawData[0], rawData[1])/16384
    accelY = ConvertBytesToInt(rawData[2], rawData[3])/16384
    accelZ = ConvertBytesToInt(rawData[4], rawData[5])/16384
    
    accelData = {'Ax':accelX,'Ay':accelY,'Az':accelZ} 
    
    return accelData

# Read user command from Thingspeak server
def ReadDataFromThingspeak():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    addressInfo = socket.getaddrinfo(thingspeakApiHost, thingspeakApiPort)
    address = addressInfo[0][-1]
    s.connect(address)
    s.send("GET https://{}/channels/{}/fields/1.json?api_key={}&results=1\r\n\r\n".format(thingspeakApiHost, thingspeakChannelId, thingspeakReadKey))
    response = str(s.recv(1024))
    response = response[2:len(response)-1]
    s.close()
    if not response:
        print("No response from Thingspeak.. Error")
        return
    
    data = json.loads(response)
    channelFeedValues = data["feeds"]
    print(channelFeedValues)
    if not channelFeedValues:
        return None 
    userCommand = channelFeedValues[0]['field1']
    return userCommand
        
# Updates the state of the Device from User command
def UpdateDeviceState(userCommand):
    global deviceState
    if userCommand  == 'activate':
        deviceState = True
        greenLed.value(1)
    elif userCommand  == 'deactivate':
        deviceState = False
        greenLed.value(0)
           
# Constantly listens to Thingspeak channel for commands from user.
def TakeActionBasedOnUserCommand(userCommand):
    global deviceState
    # Udpdate the device state
    UpdateDeviceState(userCommand)
    print(deviceState)
    
    if(deviceState == True):
        print("Motion-Detection Device Activated")
        timer2.init(period = 2000, mode = Timer.PERIODIC, callback= lambda t: CheckIfDeviceIsInMovement())
    elif(deviceState == False):
        print("Motion Detection Device Deactivated")
        timer2.deinit()
        redLed.value(0)

# Check if any motion is detected
def CheckIfDeviceIsInMovement():
    global previousAccelValues
    currentAccelValues = CalculateCorrectAccelerometerValues()
    diffInAccelValues = {}
    diffInAccelValues['Ax'] = abs(currentAccelValues['Ax']) - abs(previousAccelValues['Ax'])
    diffInAccelValues['Ay'] = abs(currentAccelValues['Ay']) - abs(previousAccelValues['Ay'])
    diffInAccelValues['Az'] = abs(currentAccelValues['Az']) - abs(previousAccelValues['Az'])
    if(diffInAccelValues['Ax'] > thresholdX or diffInAccelValues['Ay'] > thresholdY or diffInAccelValues['Az'] > thresholdZ):
        print("Movement detected")
        # Update Red Led state
        redLed.value(1)
        # Send notification via IFTTT app
        SendNotificationToUser(currentAccelValues)
    else:
        print("Movement not detected")
        redLed.value(0)
    previousAccelValues = currentAccelValues
        
def SendNotificationToUser(currentAccelValues):
    print("Sending notification to user")
    accelX = currentAccelValues['Ax']
    accelY = currentAccelValues['Ay']
    accelZ = currentAccelValues['Az']
    
    url = str('https://maker.ifttt.com/trigger/{}/with/key/{}?value1={}&value2={}&value3={}'.format(webhookEventName, webhookKey, accelX, accelY, accelZ))
    print(url)
    response = urequests.get(url)
    response.close()

# Calculate Offset Corrected Accelerometer values
def CalculateCorrectAccelerometerValues():
    #global
    correctAccelValues = {}
    accelData = GetAccelerometerValues()
    global offsetValues
    correctAccelValues['Ax'] = (accelData['Ax'] + offsetValues['Ax'])
    correctAccelValues['Ay'] = (accelData['Ay'] + offsetValues['Ay'])
    correctAccelValues['Az'] = (accelData['Az'] + offsetValues['Az'])
    print(correctAccelValues)
    
    return correctAccelValues

# Calibrate Accelerometer Sensor
def CalibrateAccelerometerSensor():
    print("Calibrating Accelerometer sensor")
    print("......")
    print("Keep the MPU6050 sensor facing upward on a flat surface..")
    global offsetValues
    offsetValues = CalculateAccelerometerOffsetValues()
    
# Convert data in bytes to integer
def ConvertBytesToInt(msbByte, lsbByte):
    if not msbByte & 0x80:
        return msbByte << 8 | lsbByte
    return (((msbByte ^ 255) << 8) | (lsbByte ^ 255) + 1)

## Main Program
    
# Enable Automatic Garbage Collection
gc.enable()

# Connect to Wireless Network
ConnectToWifi(wifi_ssid, wifi_password)

# Initialize I2C communication.
i2c = machine.I2C(scl=Pin(22), sda=Pin(23))

# Scan I2C bus to find peripheral devices.
deviceAddresses = i2c.scan();

# Print address of devices connected via I2C
PrintI2CDeviceAddress(deviceAddresses)

# Reset value of PWR_MGMT_Register in MPU6050
i2c.writeto(mpu6050Address, bytearray([107, 0]))

# Calibrate Accelerometer sensor when the device boots up
if (CheckMpu6050DeviceIsConnected):
    CalibrateAccelerometerSensor()

# Initialize Timers
# Timer to listen to Thingspeak server every 30 seconds for user commands
timer1 = Timer(0)
timer1.init(period = 30000, mode = Timer.PERIODIC, callback= lambda t: TakeActionBasedOnUserCommand(ReadDataFromThingspeak()))

# Timer to periodically record accelerometer values
timer2 = Timer(1)