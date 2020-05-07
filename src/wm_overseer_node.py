#!/usr/bin/env python

import sys
import json
import rospy
import std_msgs.msg

class overseer():
    def __init__(self,hardwareDict):
        self.hardwareDict = hardwareDict
        self.status_node  = rospy.Publisher('/'+ generalsetup['machineName'] +'/overseer/status', std_msgs.msg.Bool, queue_size=100)

    def executeFile(self,jobfilename):
        with open(jobfilename,'r') as jobfile:
            joblist = json.loads(jobfile.read())

            

if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            setupFileName = sys.argv[1]
        else:
            setupFileName = 'wm_setupFile.json'

        with open(setupFileName,'r') as setupFile:
            generalsetup, gpiosetup, pwmsetup = json.loads(setupFile.read())

        rospy.init_node(generalsetup['machineName']+'_overseer_node', anonymous=True)
        
        hardwareDict = {}
        for (nodeName,_,_,_,_,_) in pwmsetup:
            hardwareDict['pwmctrl_'+nodeName+'_setdeg'] = rospy.Publisher('/'+ generalsetup['machineName'] +'/hwctrl/pwmctrl/'+nodeName+'/setdeg', std_msgs.msg.Float64, queue_size=100)
        hardwareDict['pwmctrl_state']   = rospy.Publisher('/'+ generalsetup['machineName'] +'/hwctrl/pwmctrl/state', std_msgs.msg.Bool, queue_size=100)
        hardwareDict['valvectrl_state'] = rospy.Publisher('/'+ generalsetup['machineName'] +'/hwctrl/valvectrl/state', std_msgs.msg.Bool, queue_size=100)
        hardwareDict['pumpctrl_state']  = rospy.Publisher('/'+ generalsetup['machineName'] +'/hwctrl/pumpctrl/state', std_msgs.msg.Bool, queue_size=100)

        overseer_worker = overseer(hardwareDict)
        overseer_input  = rospy.Subscriber('/'+ generalsetup['machineName'] +'/overseer/inputfile', std_msgs.msg.Bool, overseer_worker.executeFile, queue_size=100)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
