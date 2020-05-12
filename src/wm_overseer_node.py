#!/usr/bin/env python

import sys
import json
import time
import rospy
import std_msgs.msg

class overseer():
    def __init__(self,hardwareDict,overseer_delay):
        self.hardwareDict   = hardwareDict
        self.overseer_delay = overseer_delay

    def executeFile(self,jobfilepath):
        with open(jobfilepath,'r') as jobfile:
            # joblist
            #     command:
            #         pwmSetDeg([<string:nodename>,<float:degree>])
            #         pwmActive(<bool:on/off>)
            #         valveActive(<bool:on/off>)
            #         pumpActive(<bool:on/off>)
            #         delay(<float:delat_sec>)
            #     format:
            #         [<string:command>,<list:argv>]
            joblist = json.loads(jobfile.read())

            for (cmd,argv) in joblist:
                if cmd == 'pwmSetDeg':
                    self.hardwareDict[argv[0]].publish(argv[1])
                elif cmd == 'pwmActive':
                    self.hardwareDict['pwmctrl_state'].publish(argv)
                elif cmd == 'valveActive':
                    self.hardwareDict['valvectrl_state'].publish(argv)
                elif cmd == 'pumpActive':
                    self.hardwareDict['pumpctrl_state'].publish(argv)
                elif cmd == 'delay':
                    time.sleep(argv)

                time.sleep(self.overseer_delay)

if __name__ == '__main__':
    try:
        setupFileName = sys.argv[1] if len(sys.argv) > 1 else 'wm_setupFile.json'
        with open(setupFileName,'r') as setupFile:
            generalsetup,_,pwmsetup,_,_ = json.loads(setupFile.read())

        rospy.init_node(generalsetup['machineName']+'_overseer_node', anonymous=True)
        
        hardwareDict = {}
        for (nodeName,_,_,_,_,_) in pwmsetup:
            hardwareDict['pwmctrl_'+nodeName+'_setdeg'] = rospy.Publisher('/'+ generalsetup['machineName'] +'/hwctrl/pwmctrl/'+nodeName+'/setdeg', std_msgs.msg.Float64, queue_size=100)
        hardwareDict['pwmctrl_state']   = rospy.Publisher('/'+ generalsetup['machineName'] +'/hwctrl/pwmctrl/state', std_msgs.msg.Bool, queue_size=100)
        hardwareDict['valvectrl_state'] = rospy.Publisher('/'+ generalsetup['machineName'] +'/hwctrl/valvectrl/state', std_msgs.msg.Bool, queue_size=100)
        hardwareDict['pumpctrl_state']  = rospy.Publisher('/'+ generalsetup['machineName'] +'/hwctrl/pumpctrl/state', std_msgs.msg.Bool, queue_size=100)

        overseer_worker = overseer(hardwareDict,generalsetup['overseer_delay'])
        
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/overseer/inputfile', std_msgs.msg.String, overseer_worker.executeFile, queue_size=100)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
