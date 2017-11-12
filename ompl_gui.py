from PyQt4 import QtCore,QtGui
import sys
import rospy
import numpy as np
from math import cos, sin, atan2
from InterfacePath_ompl import Ui_MainWindow
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import planner_path
from krssg_ssl_msgs.msg import point_SF
from krssg_ssl_msgs.msg import gr_Commands


points_home = []
points_home_theta = []
points_opp=[]
vrtx=[(200,200)]
curr_vel = [10,0]
VEL_UNIT = 5
BOT_ID = 0
pub = rospy.Publisher('gui_params', point_SF)

path_received=0


def debug_path(msg):
    global vrtx, path_received
    # if(path_received==1):
    #     return

    vrtx=[]
    for v in msg.point_array:
        vrtx.append(((int(v.x)),450-int(v.y)))
        # print(v.x, v.y)
    path_received=1    
    print("received path points, size = ",len(vrtx))        

              
    # cv2.imshow("bots", img)
    # print(" in display bots here")

def Callback_VelProfile(msg):
    global curr_vel
    msg = msg.robot_commands
    theta = float(points_home_theta[BOT_ID])
    # print("theta = ",theta)
    vel_theta = atan2(msg.velnormal, msg.veltangent) + theta
    vel_mag = msg.velnormal*msg.velnormal + msg.veltangent*msg.veltangent
    curr_vel = [vel_mag, vel_theta]


def Callback(msg):
    # print(" in callback")
    global points_home
    global points_home_theta
    points_home = []
    points_home_theta = []
    for i in msg.homePos:
        points_home.append([(int((i.x)+4000)*13.0/160.0), 450-int((i.y+3000)*3/40.0)])
        points_home_theta.append(i.theta)
    global points_opp    
    points_opp=[]
    for i in msg.awayPos:
        points_opp.append([(int((i.x)+4000)*13.0/160.0), 450-int((i.y+3000)*3.0/40.0)])


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

        # self.videoFrame=ImageWidget()
        # self.setCentralWidget(self.videoFrame)
        self.timer=QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(30)
        # self.capture = cv2.VideoCapture(0)
    def hide_all(self):
        # self.setCentralWidget(self)
        pass

    def show_vel_vector(self):
        global curr_vel
    
        # print("in show_vel_vector")
        print("curr_vel ", curr_vel)
        speed = curr_vel[0]
        theta = curr_vel[1]
        start_ = (points_home[0][0],points_home[0][1])
        end_ = (points_home[0][0]+VEL_UNIT*speed*cos(theta), points_home[0][1]-VEL_UNIT*speed*sin(theta))

        self.scene.addLine(start_[0],start_[1], end_[0], end_[1])
        # qp.end()


    def sendParams(self):
        print("in send_params")
        stepSize = float(self.stepSizeText.text())
        biasParam = float(self.biasParamText.text())
        maxIterations = float(self.maxIterationsText.text())
        msg=point_SF()
        msg.s_x=points_home[1][0]
        msg.s_y=points_home[1][1]
        msg.f_x=0
        msg.f_y=0
        # print(" here step_size ",stepSize)
        msg.step_size=stepSize
        msg.bias_param=biasParam
        msg.max_iteration=maxIterations
        pub.publish(msg)
        pass        

    def updateImage(self):
       
        self.display_bots(points_home, points_opp)
        self.show_vel_vector()
        # self.show_vel_vector()

    def paintEvent(self,event):
        # print(" in paint event")
        # return
        qp=QtGui.QPainter()
        qp.begin(self)
        qp.end()  

    def display_bots(self, points_home, points_opp):
        global vrtx
        self.scene.clear()
        self.graphicsView.setScene(self.scene)
        brush= QtGui.QBrush(QtCore.Qt.SolidPattern)
        print("0th bot = ",points_home[0][0], points_home[0][1])
        for point in points_home:
            self.scene.addEllipse(point[0], point[1],self.obstacleRadius,self.obstacleRadius , self.mark_e, brush)
        for point in points_opp:
            self.scene.addEllipse(point[0], point[1],self.obstacleRadius,self.obstacleRadius , self.mark_s, brush) 
        self.draw_path(vrtx)  

    def draw_path(self, vrtx):
    # print("in draw_path")
        path = QtGui.QPainterPath()
        # path_points = []
        # with open("../../../path.dat") as f:
        #     content = f.readlines()
        #     content = content[0].strip()
        #     print(content)
        #     path_points.append((int(float(content[0])/800.0*600), int(float(content[1])/500.0*400)))

        # vrtx = path_points
        # print("no of points = ", len(vrtx))  
          
        path.moveTo(vrtx[0][0],vrtx[0][1])
        print("0th position = ", vrtx[0][0], vrtx[0][1])
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
        # print(" Path added to scene now ",max_x, max_y," min = ",min_x, min_y)

app=QtGui.QApplication(sys.argv)
w=MainWindow()
def main():
    rospy.init_node('display', anonymous=True)
    rospy.Subscriber("/belief_state", BeliefState , Callback);
    rospy.Subscriber("/grsim_data", gr_Commands , Callback_VelProfile);
    rospy.Subscriber("/path_planner_ompl", planner_path, debug_path)

    w.show()
    app.exec_()

if __name__=='__main__':
    main()