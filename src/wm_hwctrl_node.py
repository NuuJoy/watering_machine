#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg
import RPi.GPIO
import Adafruit_PCA9685

class pwmDriver():
    def __init__(self,pinNum,wdth1,deg1,wdth2,deg2):
        # init self parameter
        self.pinNum = pinNum
        self.wdth1  = wdth1
        self.wdth2  = wdth2
        self.deg1   = deg1
        self.deg2   = deg2
        self._pwm = Adafruit_PCA9685.PCA9685()
        self._pwm.set_pwm_freq(50) # set frequency 50 Hz
    
    def deg2wdth_func(self,input_deg):
        # just linear interpolation from y=mx+c
        return round((self.wdth2-self.wdth1)/(self.deg2-self.deg1)*input_deg+(self.wdth1-(self.wdth2-self.wdth1)/(self.deg2-self.deg1)*self.deg1))

    def setPWM(self,rawSetDeg):
        # clamp setDeg value to limit
        setDeg = max(min(max(self.deg1,self.deg2), rawSetDeg.data), min(self.deg1,self.deg2))
        # calculate degree to int(4096)
        setInt = self.deg2wdth_func(setDeg)
        # set servo angle
        self._pwm.set_pwm(self.pinNum, 0, setInt)

class gpioDriver():
    def __init__(self,pwmena_pin,valve_pin,pump_pin):
        # init RPi.GPIO
        RPi.GPIO.setmode(RPi.GPIO.BOARD)
        RPi.GPIO.setwarnings(False)
        # pin setup
        self.pwmena_pin = pwmena_pin
        self.valve_pin  = valve_pin
        self.pump_pin   = pump_pin

    @property
    def pwmena_pin(self):
        return self.__pwmena_pin
    @pwmena_pin.setter
    def pwmena_pin(self, pwmena_pin):
        if hasattr(self,'__pwmena_pin'):
            RPi.GPIO.cleanup(self.__pwmena_pin)
        self.__pwmena_pin = pwmena_pin
        RPi.GPIO.setup(self.__pwmena_pin, RPi.GPIO.OUT, initial=RPi.GPIO.HIGH)
    @property
    def valve_pin(self):
        return self.__valve_pin
    @valve_pin.setter
    def valve_pin(self, valve_pin):
        if hasattr(self,'__valve_pin'):
            RPi.GPIO.cleanup(self.__valve_pin)
        self.__valve_pin = valve_pin
        RPi.GPIO.setup(self.__valve_pin, RPi.GPIO.OUT, initial=RPi.GPIO.LOW)
    @property
    def pump_pin(self):
        return self.__pump_pin
    @pump_pin.setter
    def pump_pin(self, pump_pin):
        if hasattr(self,'__pump_pin'):
            RPi.GPIO.cleanup(self.__pump_pin)
        self.__pump_pin = pump_pin
        RPi.GPIO.setup(self.__pump_pin, RPi.GPIO.OUT, initial=RPi.GPIO.LOW)

    def pwm_active_set(self,pwm_active_status):
        RPi.GPIO.output(self.pwmena_pin, [RPi.GPIO.HIGH,RPi.GPIO.LOW][pwm_active_status.data])  # false=high, true=low
    def valve_active_set(self,valve_active_status):
        RPi.GPIO.output(self.valve_pin, [RPi.GPIO.LOW,RPi.GPIO.HIGH][valve_active_status.data]) # false=low, true=high
    def pump_active_set(self,pump_active_status):
        RPi.GPIO.output(self.pump_pin, [RPi.GPIO.LOW,RPi.GPIO.HIGH][pump_active_status.data])   # false=low, true=high

if __name__ == '__main__':
    try:
        rospy.init_node('wm_ros_hwctrl_node', anonymous=True)

        actuatorsDict = {}

        baseazim = pwmDriver( 0, 205,-90, 410, 90)
        basealti = pwmDriver( 1, 205,-90, 410, 90)
        midnode  = pwmDriver( 2, 205,-90, 410, 90)
        endnode  = pwmDriver( 3, 205,-90, 410, 90)
        closure  = pwmDriver(15, 205,  0, 410, 90)

        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/baseazim/setdeg', std_msgs.msg.Float64, baseazim.setPWM, queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/basealti/setdeg', std_msgs.msg.Float64, basealti.setPWM, queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/midnode/setdeg' , std_msgs.msg.Float64, midnode.setPWM , queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/endnode/setdeg' , std_msgs.msg.Float64, endnode.setPWM , queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/closure/setdeg' , std_msgs.msg.Float64, closure.setPWM , queue_size=100)

        switchDict = gpioDriver(33,35,37)
        rospy.Subscriber('/wm_ros/hwctrl/pwmctrl/state'  , std_msgs.msg.Bool, switchDict.pwm_active_set  , queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/valvectrl/state', std_msgs.msg.Bool, switchDict.valve_active_set, queue_size=100)
        rospy.Subscriber('/wm_ros/hwctrl/pumpctrl/state' , std_msgs.msg.Bool, switchDict.pump_active_set , queue_size=100)

        rospy.spin()

    except rospy.ROSInterruptException:
        # clean-up pwm and gpio on exit
        for actuator in actuatorsDict:
            actuator.pwm.set_pwm(actuator.pinNum, 0, 0)
        RPi.GPIO.cleanup([switchDict._pwmenaPin,switchDict._valvePin,switchDict._pumpPin])
 
