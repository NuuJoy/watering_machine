#!/usr/bin/env python

import sys
import time
import datetime
import rospy
import RPi.GPIO
import std_msgs.msg

class hardwareControl():
    def __init__(self,pwmena_pin,valve_pin,nodeNameList):
        # pin setup
        self.pwmena_pin = pwmena_pin
        self.valve_pin  = valve_pin

        # init RPi.GPIO
        RPi.GPIO.setmode(RPi.GPIO.BOARD)
        RPi.GPIO.setwarnings(False)
        
        # init publisher
        self.pub_pwmena = rospy.Publisher('/hwctrl/pwmena', std_msgs.msg.Bool, queue_size=1)
        self.pub_valve  = rospy.Publisher('/hwctrl/valve', std_msgs.msg.Bool, queue_size=1)
        self.pub_pwmcmd = {}
        for nodeName in nodeNameList:
            self.pub_pwmcmd[nodeName] = rospy.Publisher('/hwctrl/setdeg/'+nodeName, std_msgs.msg.Float64, queue_size=1)

    def __enter__(self):
        # init with pwm disable, valve disable
        RPi.GPIO.setup(self.pwmena_pin, RPi.GPIO.OUT, initial=RPi.GPIO.HIGH)
        RPi.GPIO.setup(self.valve_pin, RPi.GPIO.OUT, initial=RPi.GPIO.LOW)
        return self
    
    def __exit__(self ,type, value, traceback):
        RPi.GPIO.cleanup([self.pwmena_pin,self.valve_pin])

    def valve_active_set(self,status):
        # HIGH = enable, LOW = disable
        if status:
            RPi.GPIO.output(self.valve_pin, RPi.GPIO.HIGH)
        else:
            RPi.GPIO.output(self.valve_pin, RPi.GPIO.LOW)
    
    def valve_active_with_delay(self,status,delay):
        RPi.GPIO.output(self.valve_pin, RPi.GPIO.HIGH)
        time.sleep(delay)
        RPi.GPIO.output(self.valve_pin, RPi.GPIO.LOW)

    def pwm_active_set(self,status):
        # LOW = enable, HIGH = disable
        if status:
            RPi.GPIO.output(self.pwmenable_pin, RPi.GPIO.LOW)
        else:
            RPi.GPIO.output(self.pwmenable_pin, RPi.GPIO.HIGH)

    def actuator_deg_set(self,nodeName,setdeg):
        self.pub_pwmcmd[nodeName].publish(setdeg)

if __name__ == '__main__':

    pwmena_pin   = int(sys.argv[0])
    valve_pin    = int(sys.argv[1])
    nodeNameList = sys.argv[2:]

    rate = rospy.Rate(rate_hz)

    try:
        with hardwareControl(pwmena_pin,valve_pin,nodeNameList) as hwctrl:
            # format: [[[datetime1],[task1,task2,...]],[[datetime2],[task1,task2,...]],...]
            managerList = [[[None,None,None,8, 30,0,0],['wakeup.json','morningwatering.json','sleep.json']],
                           [[None,None,None,13,00,0,0],['wakeup.json','afternoonwatering.json','sleep.json']],
                           [[None,None,None,16,45,0,0],['wakeup.json','eveningwatering.json','sleep.json']]]

            # datetime.datetime
            # datetime.datetime.utcnow()
            # datetime.datetime.utcnow().timestamp()
            # datetime.datetime(2020, 5, 3, 6, 57, 21, 398648)
            # datetime.datetime(2020, 5, 3, 6, 57, 21, 398648).timestamp()
        
            while not rospy.is_shutdown():
                pass
                rate.sleep()

    except rospy.ROSInterruptException:
        pass