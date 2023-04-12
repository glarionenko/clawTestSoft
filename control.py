import RPi.GPIO as GPIO
from time import sleep
import paho.mqtt.client as paho
from time import *
import os
counterPatternRepeats = 0
#GPIO configurable, motion parameters
clawMaxOperatingTime = 2.2
servoMaxOperatingTime = 7000
#test commit from rpi prod
clawDownSwitchPin = 5
clawUpSwitchPin = 7
servoRightSwitchPin = 11
servoLeftSwitchPin = 9
servoUpSwitchPin = 8
sensorSwitchPin = 26
servoLeftPin = 17
servoRightPin = 4
servoUpPin = 22
servoDownPin = 20
clawDownPin = 23
clawUpPin = 24
EMPinRelay = 6
EMPin = 12
blinkPin = 16
RPin = 13
GPin = 18
BPin = 19
pushPin = 27
pushPin2 = 21
#GPIO mode setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
#motion pins
GPIO.setup(servoLeftPin,GPIO.OUT)
GPIO.setup(servoRightPin,GPIO.OUT)
GPIO.setup(servoUpPin,GPIO.OUT)
GPIO.setup(servoDownPin,GPIO.OUT)
GPIO.setup(clawDownPin,GPIO.OUT)
GPIO.setup(clawUpPin,GPIO.OUT)
GPIO.setup(EMPin,GPIO.OUT)
GPIO.setup(EMPinRelay,GPIO.OUT)
#light
GPIO.setup(blinkPin, GPIO.OUT)
GPIO.setup(RPin, GPIO.OUT)
GPIO.setup(GPin, GPIO.OUT)
GPIO.setup(BPin, GPIO.OUT)
#End-swithes, without backward servoDownSwitchPin
GPIO.setup(clawDownSwitchPin,GPIO.IN)
GPIO.setup(clawUpSwitchPin,GPIO.IN)
GPIO.setup(servoRightSwitchPin,GPIO.IN)
GPIO.setup(servoLeftSwitchPin,GPIO.IN)
GPIO.setup(servoUpSwitchPin,GPIO.IN)
#sensor
GPIO.setup(sensorSwitchPin,GPIO.IN)

GPIO.output(blinkPin, GPIO.HIGH)
def resetAllPins():
    GPIO.output(EMPinRelay, GPIO.LOW)
    GPIO.output(servoLeftPin, GPIO.LOW)
    GPIO.output(servoRightPin, GPIO.LOW)
    GPIO.output(servoDownPin, GPIO.LOW)
    GPIO.output(servoUpPin, GPIO.LOW)
    GPIO.output(clawDownPin, GPIO.LOW)
    GPIO.output(clawUpPin, GPIO.LOW)
resetAllPins()
#catcher config
pwm = GPIO.PWM(EMPin,100)

stopAllMovmentsCommand = -1
clawCatchCommand = -2
clawReleaseCommand = -3
clawSetUpPower = -4
shutdownRuspberry = -5
pinEnabler=GPIO.HIGH
pinDisabler=GPIO.LOW
motionDictionary = {
    "claw_left" : servoLeftPin,
    "claw_right" : servoRightPin,
    "claw_forward" : servoUpPin,
    "claw_backward" : servoDownPin,
    "claw_down" : clawDownPin,
    "claw_up" : clawUpPin,
    "claw_stop" : stopAllMovmentsCommand,
    "claw_catch" : clawCatchCommand,
    "claw_release" : clawReleaseCommand,
    "setup_power": clawSetUpPower,
    "shutdown_raspberry": shutdownRuspberry
}

catchPower=0 #90
isCatchEnabled = False
def setCatchPower(powerMsg):
    global catchPower, isCatchEnabled, pwm, counterPatternRepeats
    catchPower = int(powerMsg)
    if(isCatchEnabled):
        pwm.ChangeDutyCycle(catchPower)
        counterPatternRepeats = 0

def motion(command):
    global catchPower, isCatchEnabled, pwm
    pin = motionDictionary[command]
    if(pin==-1):
        #print("GOING TO STOP")
        GPIO.output(servoLeftPin, pinDisabler)
        GPIO.output(servoRightPin, pinDisabler)
        GPIO.output(servoDownPin, pinDisabler)
        GPIO.output(servoUpPin, pinDisabler)
        GPIO.output(clawDownPin, pinDisabler)
        GPIO.output(clawUpPin, pinDisabler)
    if(pin>=0):
        GPIO.output(pin, pinEnabler)
    if(pin==-2):
        if(isCatchEnabled==False):
            GPIO.output(EMPinRelay, GPIO.HIGH)
            #BLOCKING
            sleep(0.3)
            pwm.start(catchPower)
            isCatchEnabled = True
    if(pin==-3):
        if(isCatchEnabled):
            pwm.stop()
            #BLOCKING
            sleep(0.3)
            GPIO.output(EMPinRelay, GPIO.LOW)
            isCatchEnabled = False
    if(pin==shutdownRuspberry):
        resetAllPins()
        os.system("sudo shutdown -h now")

#mqtt auth
broker="127.0.0.1"
port=1883

#mqtt handlers
def on_disconnect(client, userdata, rc):
    #print("disonnected")
    client1.connect_async()
def on_publish(client,userdata,result):             #create function for callback
    pass
    #print("data published \n")
def on_message(client, userdata, message):
    #print("message received " ,str(message.payload.decode("utf-8")))
    #print("message topic=",message.topic)
    #print("message qos=",message.qos)
    #print("message retain flag=",message.retain)
    if(message.topic=="claw/button"):
        #print("Button")
        #print(str(message.payload.decode("utf-8")))
        motion(str(message.payload.decode("utf-8")))
    if(message.topic=="claw/power"):
        #print("Power")
        #print(str(message.payload.decode("utf-8")))
        if(not(str(message.payload.decode("utf-8"))=="")):
            setCatchPower(str(message.payload.decode("utf-8")))

#mqtt connection
client1= paho.Client("localMachine")                           #create client object
client1.username_pw_set("glarionenko","55566678")
client1.on_publish = on_publish                          #assign function to callback
client1.on_message=on_message
client1.on_disconnect=on_disconnect
client1.connect(broker,port)
client1.loop_start()
client1.subscribe("claw/#")




continuouslySendSensorValues = False

endSwitchPin =[clawDownSwitchPin,clawUpSwitchPin,servoRightSwitchPin,servoLeftSwitchPin,servoUpSwitchPin]
endSwitchTopic= ["down_end","up_end", "right_end","left_end","forward_end" ]
endSwitchOldValue=[1,1,1,1,1]
endSwitchNewValue=[1,1,1,1,1]

def readEndSwitches():
    global endSwitchPressedValue, clawDownSwitchPinState,clawUpSwitchPinState,servoRightSwitchPinState 
    global servoLeftSwitchPinState, servoUpSwitchPinState, client1
    clawDownSwitchPinStateNew = GPIO.input(clawDownSwitchPin)
    for index, pin in enumerate(endSwitchPin):
       endSwitchNewValue[index] = GPIO.input(pin)
       if(endSwitchNewValue[index]!=endSwitchOldValue[index]):
           endSwitchOldValue[index]=endSwitchNewValue[index]
           client1.publish("claw/info/"+endSwitchTopic[index],str(endSwitchNewValue[index]),qos=1)





#GPIO.setup(clawDownSwitchPin,GPIO.IN)
#GPIO.setup(clawUpSwitchPin,GPIO.IN)
#GPIO.setup(servoRightSwitchPin,GPIO.IN)
#GPIO.setup(servoLeftSwitchPin,GPIO.IN)
#GPIO.setup(servoUpSwitchPin,GPIO.IN)

def firstPattern():
    global endSwitchPressedValue, clawDownSwitchPinState,clawUpSwitchPinState,servoRightSwitchPinState 
    global servoLeftSwitchPinState, servoUpSwitchPinState, counterPatternRepeats
    #опустить, сжать, 2 секунды, поднять до концевика, 4 секунды, разжать, 14секунд
    motion("claw_down")
    sleep(1)
    motion("claw_stop")
    motion("claw_catch")
    sleep(3)
    motion("claw_up")
    #print("going up")
    while(GPIO.input(clawUpSwitchPin)==GPIO.HIGH):
        pass
    #print("going up STOPPED")
    motion("claw_stop")
    sleep(4)
    motion("claw_release")
    sleep(14)
    counterPatternRepeats = counterPatternRepeats+1
    print(counterPatternRepeats)
    


try:
    while True:
        sleep(0.1)
        readEndSwitches()
        #firstPattern()

except KeyboardInterrupt:
    client1.loop_stop()
    print("STOPPING")
    resetAllPins()
    GPIO.cleanup()
