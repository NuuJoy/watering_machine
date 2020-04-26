#!/usr/bin/env python

import sys
import json
import time
import numpy
import scipy.optimize

import vispy
import vispy.app
import vispy.scene
import vispy.scene.visuals
# --------------------------------------------------------------------------------------------------------------
class servoPoint():
    def __init__(self,nodeName,initPos,initAxis,initDeg,degMin,degMax,child=[]):
        self.nodeName = nodeName
        self.curPos   = initPos
        self.curAxs   = initAxis
        self.curDeg   = initDeg
        self.degMin   = degMin
        self.degMax   = degMax
        self.child    = child
    def action(self,degree):
        if degree < self.degMin:
            degree = self.degMin
        elif degree > self.degMax:
            degree = self.degMax
        theta     = (degree-self.curDeg)/180*numpy.pi
        rotateMat = numpy.array([[numpy.cos(theta)+(self.curAxs[0]**2)*(1-numpy.cos(theta)),
                                    self.curAxs[0]*self.curAxs[1]*(1-numpy.cos(theta))-self.curAxs[2]*numpy.sin(theta),
                                    self.curAxs[0]*self.curAxs[2]*(1-numpy.cos(theta))+self.curAxs[1]*numpy.sin(theta)],
                                    [self.curAxs[0]*self.curAxs[1]*(1-numpy.cos(theta))+self.curAxs[2]*numpy.sin(theta),
                                    numpy.cos(theta)+(self.curAxs[1]**2)*(1-numpy.cos(theta)),
                                    self.curAxs[1]*self.curAxs[2]*(1-numpy.cos(theta))-self.curAxs[0]*numpy.sin(theta)],
                                    [self.curAxs[0]*self.curAxs[2]*(1-numpy.cos(theta))-self.curAxs[1]*numpy.sin(theta),
                                    self.curAxs[1]*self.curAxs[2]*(1-numpy.cos(theta))+self.curAxs[0]*numpy.sin(theta),
                                    numpy.cos(theta)+(self.curAxs[2]**2)*(1-numpy.cos(theta))]])
        for eachChild in self.child:
            deOffsetPos      = [val1-val0 for val1,val0 in zip (eachChild.curPos,self.curPos)]
            newDeOffsetPos   = numpy.matmul(rotateMat,deOffsetPos)
            eachChild.curPos = [val1+val0 for val1,val0 in zip (newDeOffsetPos,self.curPos)]
            eachChild.curAxs = numpy.matmul(rotateMat,eachChild.curAxs)
        self.curDeg = degree
# --------------------------------------------------------------------------------------------------------------
class vispyObj():
    def __init__(self,actuators,actStateList,highlightLink,trackingPoint):
        self.actuators     = actuators
        self.actStateList  = actStateList
        self.highlightLink = highlightLink
        self.trackingPoint = trackingPoint

        # initSelfGraph(self):
        self.canvas = vispy.scene.SceneCanvas(keys='interactive')
        self.canvas.size = 1200, 600
        self.canvas.show()

        self.grid = self.canvas.central_widget.add_grid()

        curstate_subplot = self.grid.add_view(row=0, col=1, row_span=3, col_span=3)
        curstate_subplot.border_color = (0.25, 0.25, 0.25, 1)
        vispy.scene.visuals.GridLines(parent=curstate_subplot.scene)
        curstate_subplot.camera.interactive = False
        curstate_subplot.bgcolor = 'black'
        curstate_subplot.camera  = 'turntable'
        curstate_subplot.camera.center   = [0,50,0]
        curstate_subplot.camera.distance = 200.0

        frontview_subplot = self.grid.add_view(row=0, col=0, row_span=1, col_span=1)
        frontview_subplot.border_color = (0.25, 0.25, 0.25, 1)
        frontview_subplot.camera = vispy.scene.PanZoomCamera(rect=(-120,-20,240,140),interactive=False)
        vispy.scene.visuals.GridLines(parent=frontview_subplot.scene)

        topview_subplot = self.grid.add_view(row=1, col=0, row_span=1, col_span=1)
        topview_subplot.border_color = (0.25, 0.25, 0.25, 1)
        topview_subplot.camera = vispy.scene.PanZoomCamera(rect=(-120,-30,240,150),interactive=False)
        vispy.scene.visuals.GridLines(parent=topview_subplot.scene)

        sideview_subplot = self.grid.add_view(row=2, col=0, row_span=1, col_span=1)
        sideview_subplot.border_color = (0.25, 0.25, 0.25, 1)
        sideview_subplot.camera = vispy.scene.PanZoomCamera(rect=(-50,-40,190,140),interactive=False)
        vispy.scene.visuals.GridLines(parent=sideview_subplot.scene)

        for eachPoint in self.actuators:
            def randomColor():
                lineColor = [numpy.random.random(),numpy.random.random(),numpy.random.random()]
                lineColor = [val-min(lineColor) for val in lineColor]
                lineColor = [val/max(lineColor) for val in lineColor]
                return lineColor
            if eachPoint.nodeName in self.trackingPoint:
                # build path
                randClr = randomColor()
                setattr(self,eachPoint.nodeName+'_pathData',[eachPoint.curPos])
                setattr(self,eachPoint.nodeName+'_pathLine3D',vispy.scene.visuals.Line(pos=numpy.array(getattr(self,eachPoint.nodeName+'_pathData'),dtype=numpy.float64),color=randClr,antialias=True,method='gl'))
                setattr(self,eachPoint.nodeName+'_pathLineFront',vispy.scene.visuals.Line(pos=numpy.array(getattr(self,eachPoint.nodeName+'_pathData'),dtype=numpy.float64)[:,[0,1]],color=randClr,antialias=True,method='gl'))
                setattr(self,eachPoint.nodeName+'_pathLineTop',vispy.scene.visuals.Line(pos=numpy.array(getattr(self,eachPoint.nodeName+'_pathData'),dtype=numpy.float64)[:,[0,2]],color=randClr,antialias=True,method='gl'))
                setattr(self,eachPoint.nodeName+'_pathLineSide',vispy.scene.visuals.Line(pos=numpy.array(getattr(self,eachPoint.nodeName+'_pathData'),dtype=numpy.float64)[:,[1,2]],color=randClr,antialias=True,method='gl'))
                curstate_subplot.add(getattr(self,eachPoint.nodeName+'_pathLine3D'))
                frontview_subplot.add(getattr(self,eachPoint.nodeName+'_pathLineFront'))
                topview_subplot.add(getattr(self,eachPoint.nodeName+'_pathLineTop'))
                sideview_subplot.add(getattr(self,eachPoint.nodeName+'_pathLineSide'))
            
            for eachChild in eachPoint.child:
                if eachPoint.nodeName+'_'+eachChild.nodeName in self.highlightLink:
                    # build line
                    randClr = randomColor()
                    setattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_3D',vispy.scene.visuals.Line(pos=numpy.array([eachPoint.curPos,eachChild.curPos],dtype=numpy.float64),color=randClr,antialias=True,method='gl'))
                    setattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Front',vispy.scene.visuals.Line(pos=numpy.array([eachPoint.curPos,eachChild.curPos],dtype=numpy.float64)[:,[0,1]],color=randClr,antialias=True,method='gl'))
                    setattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Top',vispy.scene.visuals.Line(pos=numpy.array([eachPoint.curPos,eachChild.curPos],dtype=numpy.float64)[:,[0,2]],color=randClr,antialias=True,method='gl'))
                    setattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Side',vispy.scene.visuals.Line(pos=numpy.array([eachPoint.curPos,eachChild.curPos],dtype=numpy.float64)[:,[1,2]],color=randClr,antialias=True,method='gl'))
                    curstate_subplot.add(getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_3D'))
                    frontview_subplot.add(getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Front'))
                    topview_subplot.add(getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Top'))
                    sideview_subplot.add(getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Side'))

    def update(self,ev):
        # call status-update module
        newActState = self.actStateList[0]
        # update actuators state
        for act,val in zip(self.actuators,newActState):
            act.action(val)
        # Redraw
        for eachPoint in self.actuators:
            if eachPoint.nodeName in self.trackingPoint:
                getattr(self,eachPoint.nodeName+'_pathData').append(eachPoint.curPos)
                getattr(self,eachPoint.nodeName+'_pathLine3D').set_data(numpy.array(getattr(self,eachPoint.nodeName+'_pathData'),dtype=numpy.float64))
                getattr(self,eachPoint.nodeName+'_pathLineFront').set_data(numpy.array(getattr(self,eachPoint.nodeName+'_pathData'),dtype=numpy.float64)[:,[0,1]])
                getattr(self,eachPoint.nodeName+'_pathLineTop').set_data(numpy.array(getattr(self,eachPoint.nodeName+'_pathData'),dtype=numpy.float64)[:,[0,2]])
                getattr(self,eachPoint.nodeName+'_pathLineSide').set_data(numpy.array(getattr(self,eachPoint.nodeName+'_pathData'),dtype=numpy.float64)[:,[1,2]])
            for eachChild in eachPoint.child:
                if eachPoint.nodeName+'_'+eachChild.nodeName in self.highlightLink:
                    getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_3D').set_data(numpy.array([eachPoint.curPos,eachChild.curPos],dtype=numpy.float64))
                    getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Front').set_data(numpy.array([eachPoint.curPos,eachChild.curPos],dtype=numpy.float64)[:,[0,1]])
                    getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Top').set_data(numpy.array([eachPoint.curPos,eachChild.curPos],dtype=numpy.float64)[:,[0,2]])
                    getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Side').set_data(numpy.array([eachPoint.curPos,eachChild.curPos],dtype=numpy.float64)[:,[1,2]])
        # roll actuators state element
        self.actStateList = numpy.roll(self.actStateList,-1,axis=0)
# --------------------------------------------------------------------------------------------------------------

point4 = servoPoint('point4',[ 0 , 40, 50],[1,1,1],0,-90,90)
point3 = servoPoint('point3',[ 0 , 40, 60],[1,0,0],0,-90,90,child=[point4])
point2 = servoPoint('point2',[ 0 ,-10, 60],[1,0,0],0,-90,90,child=[point3,point4])
point1 = servoPoint('point1',[ 0 ,-10, 10],[1,0,0],0,-90,90,child=[point2,point3,point4])
point0 = servoPoint('point0',[ 0 ,-10,  0],[0,0,1],0,-90,90,child=[point1,point2,point3,point4])

with open('/home/nuujoy/Desktop/workingDirectory/wateringArm/wateringArmCmdList.json','r') as inputFile:
    actStateList = json.loads(inputFile.read())

myGraph = vispyObj((point0,point1,point2,point3,point4),actStateList,
                    ['point0_point1','point1_point2','point2_point3','point3_point4'],
                    ['point2','point3','point4'])

timer = vispy.app.Timer()
timer.connect(myGraph.update)
timer.start(0.1)

if __name__ == '__main__':
    myGraph.canvas.show()
    if sys.flags.interactive == 0:
        vispy.app.run()
