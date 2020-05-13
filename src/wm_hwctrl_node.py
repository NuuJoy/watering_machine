#!/usr/bin/env python

import sys
import json
import rospy
import std_msgs.msg
import RPi.GPIO
import Adafruit_PCA9685

class pwmDriver():
    def __init__(self,nodeName,pinNum,minWdth,maxWdth,minDeg,maxDeg):
        # init self parameter
        self.nodeName = nodeName
        self.pinNum   = pinNum
        self.minWdth  = minWdth
        self.maxWdth  = maxWdth
        self.minDeg   = minDeg
        self.maxDeg   = maxDeg
        self.curDeg   = None
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50) # set frequency 50 Hz

    def setPWM(self,setDeg):
        # adjust if setDeg is exceed limit
        setDeg = self.minDeg if setDeg < self.minDeg else setDeg
        setDeg = self.maxDeg if setDeg < self.maxDeg else setDeg
        # calculate degree to int(4096)
        setPcnt  = (setDeg-self.minDeg)/(self.maxDeg-self.minDeg)
        setInput = int(setPcnt*(self.maxWdth-self.minWdth) + self.minWdth)
        # set servo angle
        self.pwm.set_pwm(self.pinNum, 0, setInput)

class gpioDriver():
    def __init__(self,pwmena_pin,valve_pin,pump_pin):
        # pin setup
        self.pwmena_pin = pwmena_pin
        self.valve_pin  = valve_pin
        self.pump_pin   = pump_pin

        # init RPi.GPIO
        RPi.GPIO.setmode(RPi.GPIO.BOARD)
        RPi.GPIO.setwarnings(False)

        # init with pwm disable, valve disable
        RPi.GPIO.setup(self.pwmena_pin, RPi.GPIO.OUT, initial=RPi.GPIO.HIGH)
        RPi.GPIO.setup(self.valve_pin, RPi.GPIO.OUT, initial=RPi.GPIO.LOW)

    def pwm_active_set(self,pwm_active_status):
        # enable = LOW
        if pwm_active_status:
            RPi.GPIO.output(self.pwmena_pin, RPi.GPIO.LOW)
        else:
            RPi.GPIO.output(self.pwmena_pin, RPi.GPIO.HIGH)

    def valve_active_set(self,valve_active_status):
        # enable = HIGH
        if valve_active_status:
            RPi.GPIO.output(self.valve_pin, RPi.GPIO.HIGH)
        else:
            RPi.GPIO.output(self.valve_pin, RPi.GPIO.LOW)

    def pump_active_set(self,pump_active_status):
        # enable = HIGH
        if pump_active_status:
            RPi.GPIO.output(self.pump_pin, RPi.GPIO.HIGH)
        else:
            RPi.GPIO.output(self.pump_pin, RPi.GPIO.LOW)

if __name__ == '__main__':

    try:
        setupFileName = sys.argv[1] if len(sys.argv) > 1 else 'wm_setupFile.json'
        with open(setupFileName,'r') as setupFile:
            generalsetup,gpiosetup,pwmsetup,_,_ = json.loads(setupFile.read())

        rospy.init_node(generalsetup['machineName']+'_hwctrl_node', anonymous=True)

        # init pwm setting for each servo node
        # setupFile - pwmsetup
        #       format: [[nodeName,pinNum,minWdth,maxWdth,minDeg,maxDeg],...]
        #      example: [['baseazim',0,205,410,-90.0,90.0],['basealti',0,205,410,-90.0,90.0],['midnode',0,205,410,-90.0,90.0],['endnode',0,205,410,-90.0,90.0]]
        actuatorsDict = {}
        for (nodeName,pinNum,minWdth,maxWdth,minDeg,maxDeg) in pwmsetup:
            actuatorsDict[nodeName] = pwmDriver(nodeName,pinNum,minWdth,maxWdth,minDeg,maxDeg)
            rospy.Subscriber('/'+ generalsetup['machineName'] +'/hwctrl/pwmctrl/'+nodeName+'/setdeg', std_msgs.msg.Float64, actuatorsDict[nodeName].setPWM, queue_size=100)
        closure = pwmDriver(nodeName,pinNum,minWdth,maxWdth,minDeg,maxDeg)
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/hwctrl/pwmctrl/closure/setdeg', std_msgs.msg.Float64, closure.setPWM, queue_size=100)

        # init gpio setting for each pin
        # setupFile - gpiosetup
        #       format: [pwmena_pin,valve_pin,pump_pin]
        #      example: [11,13,15]
        switchDict = gpioDriver(*gpiosetup)
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/hwctrl/pwmctrl/state', std_msgs.msg.Bool, switchDict.pwm_active_set, queue_size=100)
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/hwctrl/valvectrl/state', std_msgs.msg.Bool, switchDict.valve_active_set, queue_size=100)
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/hwctrl/pumpctrl/state', std_msgs.msg.Bool, switchDict.pump_active_set, queue_size=100)

        rospy.spin()

    except rospy.ROSInterruptException:
        # clean-up pwm and gpio on exit
        for actuator in actuatorsDict:
            actuator.pwm.set_pwm(actuator.pinNum, 0, 0)
        RPi.GPIO.cleanup([switchDict.pwmena_pin,switchDict.valve_pin,switchDict.pump_pin])
