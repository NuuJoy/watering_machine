#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg
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
        # init publish object
        self.pub_curDeg = rospy.Publisher('/hwctrl/curdeg/'+self.nodeName, std_msgs.msg.Float64, queue_size=1)

    def setPWM(self,setDeg):
        # adjust if setDeg is exceed limit
        setDeg = self.minDeg if setDeg < self.minDeg else setDeg
        setDeg = self.maxDeg if setDeg < self.maxDeg else setDeg
        # calculate degree to int(4096)
        setPcnt  = (setDeg-self.minDeg)/(self.maxDeg-self.minDeg)
        setInput = int(setPcnt*(self.maxWdth-self.minWdth) + self.minWdth)
        # set servo angle
        self.pwm.set_pwm(self.pinNum, 0, setInput)
        # publish output
        self.curDeg = setDeg
        self.pub_curDeg.publish(self.curDeg)

if __name__ == '__main__':
    try:
        # init rospy node
        rospy.init_node('motor_pwm_control', anonymous=True)
        # sys.argv
        # input format: nodeName pinNum minWdth maxWdth minDeg maxDeg
        #      example: baseazim 0 205 410 -90 90 basealti 0 205 410 -90 90 midarm 0 205 410 -90 90
        sensorDict = {}
        for i in range(len(sys.argv)//6):
	        nodeName = sys.argv[6*i]
            pinNum   = int(sys.argv[6*i+1])
            minWdth  = int(sys.argv[6*i+2])
            maxWdth  = int(sys.argv[6*i+3])
            minDeg   = float(sys.argv[6*i+4])
            maxDeg   = float(sys.argv[6*i+5])
            actuatorsDict[nodeName] = pwmDriver(nodeName,pinNum,minWdth,maxWdth,minDeg,maxDeg)
        # setup subscriber for every node
        for eachActuator in actuatorsDict:
            rospy.Subscriber('/hwctrl/setdeg/'+eachActuator.nodeName, std_msgs.msg.Float64, eachActuator.setPWM, queue_size=1)
        rospy.spin()

    except rospy.ROSInterruptException:
        for i in range(16):
            pwm.set_pwm(i, 0, 0)
