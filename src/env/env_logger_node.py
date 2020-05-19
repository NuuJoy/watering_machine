#!/usr/bin/env python

import sys
import time
import numpy
import rospy
import std_msgs.msg

class saveToLog():
    def __init__(self,logfilename,pub_print):
        self.logfilename = logfilename
        self.pub_print   = pub_print

    def temp1_update(self,data):
        curTime = time.time()
        self.pub_print.publish('temp1,{},{}'.format(curTime,data.data))
    def temp2_update(self,data):
        curTime = time.time()
        self.pub_print.publish('temp2,{},{}'.format(curTime,data.data))
    def humd1_update(self,data):
        curTime = time.time()
        self.pub_print.publish('humd1,{},{}'.format(curTime,data.data))
    def humd2_update(self,data):
        curTime = time.time()
        self.pub_print.publish('humd2,{},{}'.format(curTime,data.data))
    def inlx1_update(self,data):
        curTime = time.time()
        self.pub_print.publish('inlx1,{},{}'.format(curTime,data.data))
    def inlx2_update(self,data):
        curTime = time.time()
        self.pub_print.publish('inlx2,{},{}'.format(curTime,data.data))
    def write2file(self,data):
        with open(self.logfilename,'a') as logfile:
            logfile.write(data.data+'\n')

if __name__ == '__main__' and sys.flags.interactive == 0:
    try:
        # init ros node
        rospy.init_node('wm_env_monitor_node', anonymous=True)
        # init data storage
        pub_print = rospy.Publisher('env_monitor_node/print2file', std_msgs.msg.String, queue_size=100)
        log = saveToLog('/home/nuujoy/wmros_ws/logFile.csv',pub_print)
        # init subscriber
        rospy.Subscriber('env_monitor_node/temp1', std_msgs.msg.Float64, log.temp1_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/temp2', std_msgs.msg.Float64, log.temp2_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/humd1', std_msgs.msg.Float64, log.humd1_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/humd2', std_msgs.msg.Float64, log.humd2_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/inlx1', std_msgs.msg.Float64, log.inlx1_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/inlx2', std_msgs.msg.Float64, log.inlx2_update, queue_size=1)

        rospy.Subscriber('env_monitor_node/print2file', std_msgs.msg.String, log.write2file, queue_size=100)

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
