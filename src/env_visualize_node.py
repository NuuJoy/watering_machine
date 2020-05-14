#!/usr/bin/env python

import sys
import time
import numpy
import rospy
import std_msgs.msg
import vispy

history_hr = 48.0

class envDataStore():
    def __init__(self,history_hr):
        self.temp1 = []
        self.temp2 = []
        self.humd1 = []
        self.humd2 = []
        self.inlx1 = []
        self.inlx2 = []
        self.history_hr  = history_hr
        self.history_sec = 3600.0*history_hr
        # init vispy for data-visualize
        canvas = vispy.scene.SceneCanvas(keys='interactive')
        canvas.size = 1200, 600
        canvas.show()
        grid = canvas.central_widget.add_grid()
        # temperature subplot
        self.temperature_subplot = grid.add_view(row=0, col=0, row_span=1, col_span=1)
        self.temperature_subplot.border_color = (0.5, 0.5, 0.5, 1)
        self.temperature_subplot.camera = vispy.scene.PanZoomCamera(rect=(-self.history_hr,20.0,self.history_hr,20.0),interactive=True)
        vispy.scene.visuals.GridLines(parent=self.temperature_subplot.scene)
        self.temperature1_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[1,0,0], antialias=False, method='gl')
        self.temperature2_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[1,0,0], antialias=False, method='gl')
        self.temperature_subplot.add(self.temperature1_line)
        self.temperature_subplot.add(self.temperature2_line)
        # humidity subplot
        self.humidity_subplot = grid.add_view(row=1, col=0, row_span=1, col_span=1)
        self.humidity_subplot.border_color = (0.5, 0.5, 0.5, 1)
        self.humidity_subplot.camera = vispy.scene.PanZoomCamera(rect=(-self.history_hr,0.0,self.history_hr,100.0),interactive=True)
        vispy.scene.visuals.GridLines(parent=self.humidity_subplot.scene)
        self.humidity1_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[0,1,0], antialias=False, method='gl')
        self.humidity2_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[0,1,0], antialias=False, method='gl')
        self.humidity_subplot.add(self.humidity1_line)
        self.humidity_subplot.add(self.humidity2_line)
        # ambientlight subplot
        self.ambientlight_subplot = grid.add_view(row=2, col=0, row_span=1, col_span=1)
        self.ambientlight_subplot.border_color = (0.5, 0.5, 0.5, 1)
        self.ambientlight_subplot.camera = vispy.scene.PanZoomCamera(rect=(-self.history_hr,0.0,self.history_hr,5000.0),interactive=True)
        vispy.scene.visuals.GridLines(parent=self.ambientlight_subplot.scene)
        self.ambientlight1_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[0,0,1], antialias=False, method='gl')
        self.ambientlight2_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[0,0,1], antialias=False, method='gl')
        self.ambientlight_subplot.add(self.ambientlight1_line)
        self.ambientlight_subplot.add(self.ambientlight2_line)

    def temp1_update(self,data):
        self.temp1.append([time.time(),data.data])
        curTime = time.time()
        plot_temp1 = [[datapoint[0]-curTime,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in env.temp1]
        plot_temp1.remove(None)
        self.temperature1_line.set_data(pos=plot_temp1)
    def temp2_update(self,data):
        self.temp2.append([time.time(),data.data])
        curTime = time.time()
        plot_temp2 = [[datapoint[0]-curTime,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in env.temp2]
        plot_temp2.remove(None)
        self.temperature2_line.set_data(pos=plot_temp2)
    def humd1_update(self,data):
        self.humd1.append([time.time(),data.data])
        curTime = time.time()
        plot_humd1 = [[datapoint[0]-curTime,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in env.humd1]
        plot_humd1.remove(None)
        self.humidity1_line.set_data(pos=plot_humd1)
    def humd2_update(self,data):
        self.humd2.append([time.time(),data.data])
        curTime = time.time()
        plot_humd2 = [[datapoint[0]-curTime,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in env.humd2]
        plot_humd2.remove(None)
        self.humidity2_line.set_data(pos=plot_humd2)
    def inlx1_update(self,data):
        self.inlx1.append([time.time(),data.data])
        curTime = time.time()
        plot_inlx1 = [[datapoint[0]-curTime,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in env.inlx1]
        plot_inlx1.remove(None)
        self.ambientlight1_line.set_data(pos=plot_inlx1)
    def inlx2_update(self,data):
        self.inlx2.append([time.time(),data.data])
        curTime = time.time()
        plot_inlx2 = [[datapoint[0]-curTime,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in env.inlx2]
        plot_inlx2.remove(None)
        self.ambientlight2_line.set_data(pos=plot_inlx2)

if __name__ == '__main__' and sys.flags.interactive == 0:
    try:
        # init ros node
        rospy.init_node('wm_env_monitor_node', anonymous=True)
        # init data storage
        env = envDataStore(history_hr)
        # init subscriber
        rospy.Subscriber('env_monitor_node/temp1', std_msgs.msg.Float64, env.temp1_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/temp2', std_msgs.msg.Float64, env.temp2_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/humd1', std_msgs.msg.Float64, env.humd1_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/humd2', std_msgs.msg.Float64, env.humd2_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/inlx1', std_msgs.msg.Float64, env.inlx1_update, queue_size=1)
        rospy.Subscriber('env_monitor_node/inlx2', std_msgs.msg.Float64, env.inlx2_update, queue_size=1)

        while not rospy.is_shutdown():
            vispy.app.run()

    except rospy.ROSInterruptException:
        pass
