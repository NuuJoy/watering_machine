import sys
import json
import time
import numpy
import scipy.optimize

import vispy
import vispy.app
import vispy.scene
import vispy.scene.visuals

from _servoPoint import servoPoint

# --------------------------------------------------------------------------------------------------------------
class vispyObj():
    def __init__(self,actuators,actStateList,highlightLink,trackingPoint,targetList,obstacle):
        self.actuators     = actuators
        self.actStateList  = actStateList
        self.highlightLink = highlightLink
        self.trackingPoint = trackingPoint
        self.targetList    = targetList

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
        curstate_subplot.camera.center   = [120,80,0]
        curstate_subplot.camera.distance = 500.0

        frontview_subplot = self.grid.add_view(row=0, col=0, row_span=1, col_span=1)
        frontview_subplot.border_color = (0.25, 0.25, 0.25, 1)
        frontview_subplot.camera = vispy.scene.PanZoomCamera(rect=(-350,100,700,1),aspect=1,interactive=False, up='+z')
        frontTitle = vispy.scene.visuals.Text('Front view',color=[1,1,1])
        frontTitle.font_size = 10
        frontTitle.pos = [-250,-50]
        frontview_subplot.add(frontTitle)
        vispy.scene.visuals.GridLines(parent=frontview_subplot.scene)

        topview_subplot = self.grid.add_view(row=1, col=0, row_span=1, col_span=1)
        topview_subplot.border_color = (0.25, 0.25, 0.25, 1)
        topview_subplot.camera = vispy.scene.PanZoomCamera(rect=(-350,100,700,1),aspect=1,interactive=False, up='-z')
        topTitle = vispy.scene.visuals.Text('Top view',color=[1,1,1])
        topTitle.font_size = 10
        topTitle.pos = [-250,-50]
        topview_subplot.add(topTitle)
        vispy.scene.visuals.GridLines(parent=topview_subplot.scene)

        sideview_subplot = self.grid.add_view(row=2, col=0, row_span=1, col_span=1)
        sideview_subplot.border_color = (0.25, 0.25, 0.25, 1)
        sideview_subplot.camera = vispy.scene.PanZoomCamera(rect=(-350,100,700,1),aspect=1,interactive=False, up='-z')
        sideTitle = vispy.scene.visuals.Text('Side view',color=[1,1,1])
        sideTitle.font_size = 10
        sideTitle.pos = [-250,-50]
        sideview_subplot.add(sideTitle)
        vispy.scene.visuals.GridLines(parent=sideview_subplot.scene)

        # ------- draw target
        curstate_scat = vispy.scene.visuals.Markers(pos=numpy.array([target[-1] for target in targetList]), 
                                                    edge_color=[0,0,0], 
                                                    face_color=[1,1,1], 
                                                    size=5)
        curstate_subplot.add(curstate_scat)

        # ------- draw obstacle
        for eachBox in obstacle:
            x,y,z,w,d,h = eachBox
            
            obstacleBox = vispy.scene.visuals.Box(width=w, height=h, depth=d,color=[0.25,0.25,0.25],edge_color=[1,1,1])
            obstacleBox.transform = vispy.visuals.transforms.MatrixTransform()
            obstacleBox.transform.translate([x,y,z+(h/2)])
            curstate_subplot.add(obstacleBox)

            obstacleBox = vispy.scene.visuals.Box(width=w, height=d, depth=h,color=[0.25,0.25,0.25],edge_color=[1,1,1])
            obstacleBox.transform = vispy.visuals.transforms.MatrixTransform()
            obstacleBox.transform.translate([x,z+(h/2),y])
            frontview_subplot.add(obstacleBox)

            obstacleBox = vispy.scene.visuals.Box(width=w, height=h, depth=d,color=[0.25,0.25,0.25],edge_color=[1,1,1])
            obstacleBox.transform = vispy.visuals.transforms.MatrixTransform()
            obstacleBox.transform.translate([x,y,z+(h/2)])
            topview_subplot.add(obstacleBox)

            obstacleBox = vispy.scene.visuals.Box(width=d, height=w, depth=h,color=[0.25,0.25,0.25],edge_color=[1,1,1])
            obstacleBox.transform = vispy.visuals.transforms.MatrixTransform()
            obstacleBox.transform.translate([y,z+(h/2),x])
            sideview_subplot.add(obstacleBox)

        # -------- draw linkage and path
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
                setattr(self,eachPoint.nodeName+'_pathLine3D',vispy.scene.visuals.Line(pos=numpy.array(getattr(self,eachPoint.nodeName+'_pathData')),color=randClr,antialias=True,method='gl'))
                setattr(self,eachPoint.nodeName+'_pathLineFront',vispy.scene.visuals.Line(pos=numpy.array(getattr(self,eachPoint.nodeName+'_pathData'))[:,[0,2,1]],color=randClr,antialias=True,method='gl'))
                setattr(self,eachPoint.nodeName+'_pathLineTop',vispy.scene.visuals.Line(pos=numpy.array(getattr(self,eachPoint.nodeName+'_pathData'))[:,[0,1,2]],color=randClr,antialias=True,method='gl'))
                setattr(self,eachPoint.nodeName+'_pathLineSide',vispy.scene.visuals.Line(pos=numpy.array(getattr(self,eachPoint.nodeName+'_pathData'))[:,[1,2,0]],color=randClr,antialias=True,method='gl'))
                curstate_subplot.add(getattr(self,eachPoint.nodeName+'_pathLine3D'))
                frontview_subplot.add(getattr(self,eachPoint.nodeName+'_pathLineFront'))
                topview_subplot.add(getattr(self,eachPoint.nodeName+'_pathLineTop'))
                sideview_subplot.add(getattr(self,eachPoint.nodeName+'_pathLineSide'))
            
            for eachChild in eachPoint.child:
                if eachPoint.nodeName+'_'+eachChild.nodeName in self.highlightLink:
                    # build line
                    randClr = randomColor()
                    setattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_3D',vispy.scene.visuals.Line(pos=numpy.array([eachPoint.curPos,eachChild.curPos]),color=randClr,antialias=True,method='gl'))
                    setattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Front',vispy.scene.visuals.Line(pos=numpy.array([eachPoint.curPos,eachChild.curPos])[:,[0,2,1]],color=randClr,antialias=True,method='gl'))
                    setattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Top',vispy.scene.visuals.Line(pos=numpy.array([eachPoint.curPos,eachChild.curPos])[:,[0,1,2]],color=randClr,antialias=True,method='gl'))
                    setattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Side',vispy.scene.visuals.Line(pos=numpy.array([eachPoint.curPos,eachChild.curPos])[:,[1,2,0]],color=randClr,antialias=True,method='gl'))
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
                getattr(self,eachPoint.nodeName+'_pathLine3D').set_data(numpy.array(getattr(self,eachPoint.nodeName+'_pathData')))
                getattr(self,eachPoint.nodeName+'_pathLineFront').set_data(numpy.array(getattr(self,eachPoint.nodeName+'_pathData'))[:,[0,2,1]])
                getattr(self,eachPoint.nodeName+'_pathLineTop').set_data(numpy.array(getattr(self,eachPoint.nodeName+'_pathData'))[:,[0,1,2]])
                getattr(self,eachPoint.nodeName+'_pathLineSide').set_data(numpy.array(getattr(self,eachPoint.nodeName+'_pathData'))[:,[1,2,0]])
            for eachChild in eachPoint.child:
                if eachPoint.nodeName+'_'+eachChild.nodeName in self.highlightLink:
                    getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_3D').set_data(numpy.array([eachPoint.curPos,eachChild.curPos]))
                    getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Front').set_data(numpy.array([eachPoint.curPos,eachChild.curPos])[:,[0,2,1]])
                    getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Top').set_data(numpy.array([eachPoint.curPos,eachChild.curPos])[:,[0,1,2]])
                    getattr(self,eachPoint.nodeName+'_'+eachChild.nodeName+'_Side').set_data(numpy.array([eachPoint.curPos,eachChild.curPos])[:,[1,2,0]])
        # roll actuators state element
        self.actStateList = numpy.roll(self.actStateList,-1,axis=0)
# --------------------------------------------------------------------------------------------------------------

with open('wateringArmCmdList.json','r') as inputFile:
    initNode,actStateList,targetList = json.loads(inputFile.read())


# build-up geometry
buildNode = {}
inputNode = []
for eachInitNode in initNode:
    buildNode[eachInitNode[0]] = servoPoint(eachInitNode[0],eachInitNode[1],eachInitNode[2],eachInitNode[3],eachInitNode[4],eachInitNode[5],[buildNode[eachName] for eachName in eachInitNode[6]])
    inputNode.append(buildNode[eachInitNode[0]])
inputNode.reverse()

obstacle = [[120,80,0,240,160,5],
            [80,80,5,140,140,10],[80,80,5,140,140,10],[80,15,15,140,10,35],[80,145,15,140,10,35],[15,80,15,10,120,35],[145,80,15,10,120,35],
            [185,105,50,90,90,10],[185,65,60,90,10,30],[185,145,60,90,10,30],[145,105,60,10,70,30],[225,105,60,10,70,30],
            [225,65,5,10,10,45],[225,145,5,10,10,45]]

myGraph = vispyObj(actuators=inputNode,
                   actStateList=actStateList,
                   highlightLink=['point0_point1','point1_point2','point2_point3','point3_point4'],
                   trackingPoint=['point2','point3','point4'],
                   targetList=targetList,
                   obstacle=obstacle)

timer = vispy.app.Timer()
timer.connect(myGraph.update)
timer.start(0.1)

if __name__ == '__main__':
    myGraph.canvas.show()
    if sys.flags.interactive == 0:
        vispy.app.run()
