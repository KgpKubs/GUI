from PyQt4 import QtCore,QtGui
import sys
import rospy
import numpy as np
from math import cos, sin, atan2, sqrt
from InterfacePath_ompl import Ui_MainWindow
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import planner_path
from krssg_ssl_msgs.msg import point_SF
from krssg_ssl_msgs.msg import gr_Commands


points_home = []
points_home_theta = []
points_opp=[]
expectedPos = [0,0]
homeBotPos = [0,0]
vrtx=[(200,200)]
curr_vel = [10,0]
VEL_UNIT = 5
BOT_ID = 0
pub = rospy.Publisher('gui_params', point_SF)
path_received=0




global flag
flag = 0
def debug_path(msg):
    global flag
    global expectedPos
    global homeBotPos
    if sqrt((expectedPos[0] - homeBotPos[0])**2 + (expectedPos[1] - homeBotPos[1])**2) > 500:
        flag = 0
    if(flag==1):
        return
    print("Here")
    global vrtx, path_received
    vrtx=[]
    for v in msg.point_array:
        vrtx.append(((int(v.x)),450-int(v.y)))
    path_received=1
    flag=1

def Callback_VelProfile(msg):
    print("OMPL subscribed with grsimData")
    global curr_vel
    global expectedPos
    msg = msg.robot_commands
    theta = float(points_home_theta[BOT_ID])
    vel_theta = atan2(msg.velnormal, msg.veltangent) + theta
    ###############################
    # CHECK
    vel_mag = sqrt(msg.velnormal*msg.velnormal + msg.veltangent*msg.veltangent)
    ################################
    curr_vel = [vel_mag, vel_theta]
    expectedPos = [msg.nextExpectedX,msg.nextExpectedY]


def Callback(msg):
    # print("BeliefState OK")
    global points_home
    global points_home_theta
    global homeBotPos
    global flag
    global expectedPos
    points_home = []
    points_home_theta = []
    homeBotPos = [msg.homePos[0].x,msg.homePos[0].y]
    for i in msg.homePos:
        points_home.append([(int((i.x)+4000)*13.0/160.0), 450-int((i.y+3000)*3/40.0)])
        points_home_theta.append(i.theta)

    global points_opp    
    points_opp=[]
    for i in msg.awayPos:
        points_opp.append([(int((i.x)+4000)*13.0/160.0), 450-int((i.y+3000)*3.0/40.0)])
    if flag == 0:
        expectedPos = [msg.homePos[0].x,msg.homePos[0].y]

class MainWindow(QtGui.QMainWindow, Ui_MainWindow, QtGui.QWidget):
    def __init__(self,parent=None):
        super(MainWindow,self).__init__(parent)
        QtGui.QWidget.__init__(self)
        self.setupUi(self)
        self.scene = QtGui.QGraphicsScene()
        self.image = None
        self.sendData.clicked.connect(self.sendParams)
        #self.updatePath.clicked.connect(self.update_path)
        self.refresh.clicked.connect(self.hide_all)
        self.obstacleRadius = 10
        self.graphicsView.setFixedSize(650,450)
        self.scene.setSceneRect(0, 0, 600, 400)
        self.graphicsView.setScene(self.scene)
        self.hide_all()
        self.pen = QtGui.QPen(QtCore.Qt.green)
        self.mark_s = QtGui.QPen(QtCore.Qt.red)
        self.mark_e = QtGui.QPen(QtCore.Qt.blue)
        self.timer=QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(30)
    def hide_all(self):
        pass

    def show_vel_vector(self):
        global curr_vel
        # print("curr_vel ", curr_vel)
        speed = curr_vel[0]
        theta = curr_vel[1]
        start_ = (vrtx[0][0],vrtx[0][1])
        end_ = (vrtx[0][0]+VEL_UNIT*speed*cos(theta), vrtx[0][1]-VEL_UNIT*speed*sin(theta))

        self.scene.addLine(start_[0],start_[1], end_[0], end_[1])


    def sendParams(self):
        stepSize = float(self.stepSizeText.text())
        biasParam = float(self.biasParamText.text())
        maxIterations = float(self.maxIterationsText.text())
        msg=point_SF()
        msg.s_x=points_home[1][0]
        msg.s_y=points_home[1][1]
        msg.f_x=0
        msg.f_y=0
        msg.step_size=stepSize
        msg.bias_param=biasParam
        msg.max_iteration=maxIterations
        pub.publish(msg)
        pass        

    def updateImage(self):
       
        self.display_bots(points_home, points_opp)
        self.show_vel_vector()

    def paintEvent(self,event):
        qp=QtGui.QPainter()
        qp.begin(self)
        qp.end()  

    def display_bots(self, points_home, points_opp):
        global vrtx
        self.scene.clear()
        self.graphicsView.setScene(self.scene)
        brush= QtGui.QBrush(QtCore.Qt.SolidPattern)
        for point in points_home:
            self.scene.addEllipse(point[0], point[1],self.obstacleRadius,self.obstacleRadius , self.mark_e, brush)
        for point in points_opp:
            self.scene.addEllipse(point[0], point[1],self.obstacleRadius,self.obstacleRadius , self.mark_s, brush) 
        self.draw_path(vrtx)  

    def draw_path(self, vrtx):
        path = QtGui.QPainterPath()
        path.moveTo(vrtx[0][0],vrtx[0][1])
        max_x=0
        max_y=0
        min_x=999
        min_y=999
        for i in vrtx[1::5]:
            path.lineTo(i[0],i[1])
            if(i[0]>max_x):
                max_x=i[0]
            if(i[0]<min_x):
                min_x=i[0]

            if(i[1]>max_y):
                max_y=i[1] 
            if(i[1]<min_y):
                min_y=i[1]       
        
        self.scene.addPath(path)
app=QtGui.QApplication(sys.argv)
w=MainWindow()
def main():
    rospy.init_node('display', anonymous=False)
    rospy.Subscriber("/belief_state", BeliefState , Callback);
    rospy.Subscriber("/grsim_data", gr_Commands , Callback_VelProfile);
    rospy.Subscriber("/path_planner_ompl", planner_path, debug_path)

    w.show()
    app.exec_()

if __name__=='__main__':
    main()