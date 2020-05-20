#!/usr/bin/env python

import sys
import json
import time
import rospy
import std_msgs.msg

class overseer():
    def __init__(self,hardwareDict,loop_delay):
        self.hardwareDict = hardwareDict
        self.loop_delay   = loop_delay

    def executeFile(self,jobfilepath):
        with open(jobfilepath.data,'r') as jobfile:
            joblist = json.loads(jobfile.read())
            for (cmd,argv) in joblist:
                if cmd in hardwareDict:
                    hardwareDict[cmd].publish(argv)
                time.sleep(self.loop_delay)

if __name__ == '__main__':
    try:
        rospy.init_node('wm_ros_overseer_node', anonymous=True)
        
        hardwareDict = {}
        hardwareDict['pwmctrl_baseazim_setdeg'] = rospy.Publisher('/wm_ros/hwctrl/pwmctrl/baseazim/setdeg', std_msgs.msg.Float64, queue_size=100)
        hardwareDict['pwmctrl_basealti_setdeg'] = rospy.Publisher('/wm_ros/hwctrl/pwmctrl/basealti/setdeg', std_msgs.msg.Float64, queue_size=100)
        hardwareDict['pwmctrl_midnode_setdeg']  = rospy.Publisher('/wm_ros/hwctrl/pwmctrl/midnode/setdeg' , std_msgs.msg.Float64, queue_size=100)
        hardwareDict['pwmctrl_endnode_setdeg']  = rospy.Publisher('/wm_ros/hwctrl/pwmctrl/endnode/setdeg' , std_msgs.msg.Float64, queue_size=100)
        hardwareDict['pwmctrl_closure_setdeg']  = rospy.Publisher('/wm_ros/hwctrl/pwmctrl/closure/setdeg', std_msgs.msg.Float64, queue_size=100)

        hardwareDict['pwmctrl_state']   = rospy.Publisher('/wm_os/hwctrl/pwmctrl/state', std_msgs.msg.Bool, queue_size=100)
        hardwareDict['valvectrl_state'] = rospy.Publisher('/wm_ros/hwctrl/valvectrl/state', std_msgs.msg.Bool, queue_size=100)
        hardwareDict['pumpctrl_state']  = rospy.Publisher('/wm_ros/hwctrl/pumpctrl/state', std_msgs.msg.Bool, queue_size=100)

        overseer_worker = overseer(hardwareDict,loop_delay=0.01)
        
        rospy.Subscriber('/wm_ros/overseer/inputfile', std_msgs.msg.String, overseer_worker.executeFile, queue_size=100)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
 
