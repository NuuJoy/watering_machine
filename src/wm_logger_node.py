#!/usr/bin/env python

import sys
import time
import json
import rospy
import std_msgs.msg

class logfile_printer():
    def __init__(self,outputfile):
        self.outputfile = outputfile
        self.temp1 = None
        self.temp2 = None
        self.humd1 = None
        self.humd2 = None
        self.inlx1 = None
        self.inlx2 = None
    def env_update(self,msgs,argv):
        setattr(self,argv,round(msgs.data,1))

    @property
    def strTime(self):
        return time.strftime('%a %d %b %Y %H:%M:%S', time.localtime())
        
    def output2File(self,text):
        print(text)
        with open(self.outputfile,'a') as logfile:
            logfile.write(text)
    def overseer_cmd_print(self,msgs):
        output = json.dumps({'rawTime':time.time(),'strTime':self.strTime,'overseer_cmd':msgs.data})+'\n'
        self.output2File(output)
    def current_env_print(self):
        output = json.dumps({'rawTime':time.time(),'strTime':self.strTime,'temp1':self.temp1,'temp2':self.temp2,
                                                                          'humd1':self.humd1,'humd2':self.humd2,
                                                                          'inlx1':self.inlx1,'inlx2':self.inlx2})+'\n'
        self.output2File(output)

if __name__ == '__main__':
    try:
        rospy.init_node('wm_ros_logger_node', anonymous=True)

        printer = logfile_printer('/home/nuujoy/wmros_ws/logFile.jsav')

        rospy.Subscriber('/wm_ros/envsensor/temp1', std_msgs.msg.Float64, printer.env_update, callback_args=('temp1'), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/humd1', std_msgs.msg.Float64, printer.env_update, callback_args=('humd1'), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/inlx1', std_msgs.msg.Float64, printer.env_update, callback_args=('inlx1'), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/temp2', std_msgs.msg.Float64, printer.env_update, callback_args=('temp2'), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/humd2', std_msgs.msg.Float64, printer.env_update, callback_args=('humd2'), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/inlx2', std_msgs.msg.Float64, printer.env_update, callback_args=('inlx2'), queue_size=1)
        rospy.Subscriber('/wm_ros/overseer/inputfile', std_msgs.msg.String, printer.overseer_cmd_print, queue_size=1)

        while not rospy.is_shutdown():
            printer.current_env_print()
            rospy.sleep(60)
        
    except rospy.ROSInterruptException:
        pass
