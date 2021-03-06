#!/usr/bin/env python

import sys
import time
import json
import numpy
import rospy
import std_msgs.msg
import vispy
import vispy.scene
import vispy.app

class vispygraph():
    def __init__(self,logfilename,history_hr):
        self.temp1 = []
        self.temp2 = []
        self.humd1 = []
        self.humd2 = []
        self.inlx1 = []
        self.inlx2 = []

        self.history_hr  = history_hr
        self.history_sec = 3600.0*history_hr
        
        try:
            with open(logfilename,'r') as logfile:
                line = logfile.readline().replace('\n','')
                while line:
                    try:
                        data = json.loads(line)
                        if (time.time()-data['rawTime']) < self.history_sec:
                            for key,val in data.items():
                                if hasattr(self,key) and (val!=None):
                                    getattr(self,key).append([data['rawTime'],val])
                    except:
                        pass
                    line = logfile.readline().replace('\n','')
        except:
            pass
        
        # init vispy for data-visualize
        canvas = vispy.scene.SceneCanvas(keys='interactive')
        canvas.size = 600, 300
        canvas.show()
        grid = canvas.central_widget.add_grid()
        # temperature subplot
        self.temperature_subplot = grid.add_view(row=0, col=0, row_span=1, col_span=1)
        self.temperature_subplot.border_color = (0.5, 0.5, 0.5, 1)
        self.temperature_subplot.camera = vispy.scene.PanZoomCamera(rect=(-self.history_hr,15.0,self.history_hr,25.0),interactive=False)
        self.temperature_subplot.add(vispy.scene.visuals.GridLines())
        self.temperature_subplot.add(vispy.scene.visuals.Text(text='  Temperature',pos=[-self.history_hr,40],color='red',font_size=10,anchor_x='left',anchor_y='bottom'))
        self.temperature_subplot.add(vispy.scene.visuals.Text(text='  35',pos=[-self.history_hr,35],color='red',font_size=10,anchor_x='left',anchor_y='center'))
        self.temperature_subplot.add(vispy.scene.visuals.Text(text='  30',pos=[-self.history_hr,30],color='red',font_size=10,anchor_x='left',anchor_y='center'))
        self.temperature_subplot.add(vispy.scene.visuals.Text(text='  25',pos=[-self.history_hr,25],color='red',font_size=10,anchor_x='left',anchor_y='center'))
        self.temperature_subplot.add(vispy.scene.visuals.Text(text='  20',pos=[-self.history_hr,20],color='red',font_size=10,anchor_x='left',anchor_y='center'))
        self.temperature1_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[1,0,0], antialias=False, method='gl')
        self.temperature2_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[1,0,0], antialias=False, method='gl')
        self.temperature_subplot.add(self.temperature1_line)
        self.temperature_subplot.add(self.temperature2_line)
        # humidity subplot
        self.humidity_subplot = grid.add_view(row=1, col=0, row_span=1, col_span=1)
        self.humidity_subplot.border_color = (0.5, 0.5, 0.5, 1)
        self.humidity_subplot.camera = vispy.scene.PanZoomCamera(rect=(-self.history_hr,0.0,self.history_hr,100.0),interactive=False)
        self.humidity_subplot.add(vispy.scene.visuals.GridLines())
        self.humidity_subplot.add(vispy.scene.visuals.Text(text='  Humidity',pos=[-self.history_hr,100],color='green',font_size=10,anchor_x='left',anchor_y='bottom'))
        self.humidity_subplot.add(vispy.scene.visuals.Text(text='  80',pos=[-self.history_hr,80],color='green',font_size=10,anchor_x='left',anchor_y='center'))
        self.humidity_subplot.add(vispy.scene.visuals.Text(text='  60',pos=[-self.history_hr,60],color='green',font_size=10,anchor_x='left',anchor_y='center'))
        self.humidity_subplot.add(vispy.scene.visuals.Text(text='  40',pos=[-self.history_hr,40],color='green',font_size=10,anchor_x='left',anchor_y='center'))
        self.humidity_subplot.add(vispy.scene.visuals.Text(text='  20',pos=[-self.history_hr,20],color='green',font_size=10,anchor_x='left',anchor_y='center'))
        self.humidity1_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[0,1,0], antialias=False, method='gl')
        self.humidity2_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[0,1,0], antialias=False, method='gl')
        self.humidity_subplot.add(self.humidity1_line)
        self.humidity_subplot.add(self.humidity2_line)
        # ambientlight subplot
        self.ambientlight_subplot = grid.add_view(row=2, col=0, row_span=1, col_span=1)
        self.ambientlight_subplot.border_color = (0.5, 0.5, 0.5, 1)
        self.ambientlight_subplot.camera = vispy.scene.PanZoomCamera(rect=(-self.history_hr,-1,self.history_hr,6),interactive=False)
        self.ambientlight_subplot.add(vispy.scene.visuals.GridLines())
        self.ambientlight_subplot.add(vispy.scene.visuals.Text(text='  AmbientLight',pos=[-self.history_hr,5],color='blue',font_size=10,anchor_x='left',anchor_y='bottom'))
        self.ambientlight_subplot.add(vispy.scene.visuals.Text(text='  10000',pos=[-self.history_hr,4],color='blue',font_size=10,anchor_x='left',anchor_y='center'))
        self.ambientlight_subplot.add(vispy.scene.visuals.Text(text='  1000',pos=[-self.history_hr,3],color='blue',font_size=10,anchor_x='left',anchor_y='center'))
        self.ambientlight_subplot.add(vispy.scene.visuals.Text(text='  100',pos=[-self.history_hr,2],color='blue',font_size=10,anchor_x='left',anchor_y='center'))
        self.ambientlight_subplot.add(vispy.scene.visuals.Text(text='  10',pos=[-self.history_hr,1],color='blue',font_size=10,anchor_x='left',anchor_y='center'))
        self.ambientlight_subplot.add(vispy.scene.visuals.Text(text='  1',pos=[-self.history_hr,0],color='blue',font_size=10,anchor_x='left',anchor_y='center'))
        self.ambientlight1_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[0,0,1], antialias=False, method='gl')
        self.ambientlight2_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0]]),color=[0,0,1], antialias=False, method='gl')
        self.ambientlight_subplot.add(self.ambientlight1_line)
        self.ambientlight_subplot.add(self.ambientlight2_line)

    def temp1_update(self,data):
        self.temp1.append([time.time(),data.data])
        curTime = time.time()
        plot_temp1 = [[(datapoint[0]-curTime)/3600.0,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in self.temp1]
        try:
            while True:
                plot_temp1.remove(None)
        except:
            pass
        self.temperature1_line.set_data(pos=numpy.array(plot_temp1))
        print('{} | {} | {}'.format(time.strftime("%b %d %Y %H:%M:%S", time.localtime()),'temp1_update',data.data))
    def temp2_update(self,data):
        self.temp2.append([time.time(),data.data])
        curTime = time.time()
        plot_temp2 = [[(datapoint[0]-curTime)/3600.0,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in self.temp2]
        try:
            while True:
                plot_temp2.remove(None)
        except:
            pass
        self.temperature2_line.set_data(pos=numpy.array(plot_temp2))
        print('{} | {} | {}'.format(time.strftime("%b %d %Y %H:%M:%S", time.localtime()),'temp2_update',data.data))
    def humd1_update(self,data):
        self.humd1.append([time.time(),data.data])
        curTime = time.time()
        plot_humd1 = [[(datapoint[0]-curTime)/3600.0,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in self.humd1]
        try:
            while True:
                plot_humd1.remove(None)
        except:
            pass
        self.humidity1_line.set_data(pos=numpy.array(plot_humd1))
        print('{} | {} | {}'.format(time.strftime("%b %d %Y %H:%M:%S", time.localtime()),'humd1_update',data.data))
    def humd2_update(self,data):
        self.humd2.append([time.time(),data.data])
        curTime = time.time()
        plot_humd2 = [[(datapoint[0]-curTime)/3600.0,datapoint[1]] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in self.humd2]
        try:
            while True:
                plot_humd2.remove(None)
        except:
            pass
        self.humidity2_line.set_data(pos=numpy.array(plot_humd2))
        print('{} | {} | {}'.format(time.strftime("%b %d %Y %H:%M:%S", time.localtime()),'humd2_update',data.data))
    def inlx1_update(self,data):
        self.inlx1.append([time.time(),data.data])
        curTime = time.time()
        plot_inlx1 = [[(datapoint[0]-curTime)/3600.0,numpy.log10(datapoint[1] if datapoint[1] > 0.1 else 0.1)] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in self.inlx1]
        try:
            while True:
                plot_inlx1.remove(None)
        except:
            pass
        self.ambientlight1_line.set_data(pos=numpy.array(plot_inlx1))
        print('{} | {} | {}'.format(time.strftime("%b %d %Y %H:%M:%S", time.localtime()),'inlx1_update',data.data))
    def inlx2_update(self,data):
        self.inlx2.append([time.time(),data.data])
        curTime = time.time()
        plot_inlx2 = [[(datapoint[0]-curTime)/3600.0,numpy.log10(datapoint[1] if datapoint[1] > 0.1 else 0.1)] if (datapoint[0]-curTime) > -self.history_sec else None for datapoint in self.inlx2]
        try:
            while True:
                plot_inlx2.remove(None)
        except:
            pass
        self.ambientlight2_line.set_data(pos=numpy.array(plot_inlx2))
        print('{} | {} | {}'.format(time.strftime("%b %d %Y %H:%M:%S", time.localtime()),'inlx2_update',data.data))

if __name__ == '__main__' and sys.flags.interactive == 0:
    try:
        # init ros node
        rospy.init_node('wm_envvisual_node', anonymous=True)
        # init data storage
        plot = vispygraph('/home/nuujoy/wmros_ws/logFile.jsav',float(sys.argv[1]))
        # init subscriber
        rospy.Subscriber('/wm_ros/envsensor/temp1', std_msgs.msg.Float64, plot.temp1_update, queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/temp2', std_msgs.msg.Float64, plot.temp2_update, queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/humd1', std_msgs.msg.Float64, plot.humd1_update, queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/humd2', std_msgs.msg.Float64, plot.humd2_update, queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/inlx1', std_msgs.msg.Float64, plot.inlx1_update, queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/inlx2', std_msgs.msg.Float64, plot.inlx2_update, queue_size=1)
        
        while not rospy.is_shutdown():
            vispy.app.run()

    except rospy.ROSInterruptException:
        pass
