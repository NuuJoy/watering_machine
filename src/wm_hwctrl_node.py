#!/usr/bin/env python

import sys
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

    def setPWM(self,rawSetDeg):
        # adjust if setDeg is exceed limit
        setDeg = self.minDeg if rawSetDeg.data < self.minDeg else rawSetDeg.data
        setDeg = self.maxDeg if rawSetDeg.data > self.maxDeg else rawSetDeg.data
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
        rospy.init_node('wm_ros_hwctrl_node', anonymous=True)

        actuatorsDict = {}

        baseazim = pwmDriver('baseazim', 0, 205, 410, -90, 90)
        basealti = pwmDriver('basealti', 1, 205, 410, -90, 90)
        midnode  = pwmDriver('midnode' , 2, 205, 410, -90, 90)
        endnode  = pwmDriver('endnode' , 3, 205, 410, -90, 90)
        closure  = pwmDriver('closure' ,15, 205, 410,   0, 90)

        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/baseazim/setdeg', std_msgs.msg.Float64, baseazim.setPWM, queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/basealti/setdeg', std_msgs.msg.Float64, basealti.setPWM, queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/midnode/setdeg' , std_msgs.msg.Float64, midnode.setPWM , queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/endnode/setdeg' , std_msgs.msg.Float64, endnode.setPWM , queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/closure/setdeg' , std_msgs.msg.Float64, closure.setPWM , queue_size=100)

        switchDict = gpioDriver(27,28,29)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/state'  , std_msgs.msg.Bool, switchDict.pwm_active_set  , queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/valvectrl/state', std_msgs.msg.Bool, switchDict.valve_active_set, queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pumpctrl/state' , std_msgs.msg.Bool, switchDict.pump_active_set , queue_size=100)

        rospy.spin()

    except rospy.ROSInterruptException:
        # clean-up pwm and gpio on exit
        for actuator in actuatorsDict:
            actuator.pwm.set_pwm(actuator.pinNum, 0, 0)
        RPi.GPIO.cleanup([switchDict.pwmena_pin,switchDict.valve_pin,switchDict.pump_pin])
