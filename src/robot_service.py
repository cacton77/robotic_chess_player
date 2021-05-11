#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
import numpy as np
from numpy.linalg import inv,norm
from robot_manipulator import *
from vision_detector import *
from transformation import Trans3D

#message from ai node would have a length 4 need to processed
class RobotService:
    
    def __init__(self):

        self.server = rospy.Service('robot_service',TaskPlanning,self.serviceHandler)
        self.manipulator = RobotManipulator()
        self.detector = VisionDetector()

        self.camera_matrix = np.array([1353.131570942828, 0, 758.6928458558336, 0, 1353.743967167117, 557.9749908957598, 0, 0, 1]).reshape((3,3))
        cam_rot = np.array([-0.00285051, -0.000809386, 0.00617178, 0.999977])
        cam_trans = np.array([0.001514679603077936, -0.08438965970995699, 0.09423193500454446])
        self.TCP2camera_pose = Trans3D.from_quaternion(cam_rot, cam_trans)

        self.board = None
        self.base2chessboard_pose = Trans3D.from_quaternion(np.array([0.97388685, -0.00873453, -0.00596293, -0.22678747]),np.array([-0.29340047, 0.61994829, 0.18277942]))
    def serviceHandler(self,msg):
        rospy.loginfo("Request: {}".format(msg.request))
        if msg.request == "to general standby":
            self.manipulator.goToJointState([90,-135,90,-70,-90,0.0])
            return TaskPlanningResponse('Robot arrive general standby position') 
        
        elif msg.request == 'detect chessboard':
            feedback = self.detectChessboard()
            return TaskPlanningResponse(feedback)
        
        elif msg.request == 'chessboard state':
            fen_string,self.chessboard = self.chessboardState()
            return TaskPlanningResponse(fen_string)

        elif msg.request[:4] == 'step':
            detail = msg.request.split(':')
            self.carryOutOrder(detail[1])
            return TaskPlanningResponse('Done')

    def detectChessboard(self):
        self.manipulator.goToJointState([90,-135,90,-70,-90,0.0])
        image = self.takeImage('standby.jpg')
        base2TCP_pose = self.manipulator.robotCurrentPose()
        base2chessboard_pose = self.detector.chessboardPose(image,base2TCP_pose)
        self.__takeImagePose(base2chessboard_pose)
        self.manipulator.goStraightToPose([self.camera_pose])
        base2TCP_pose = self.manipulator.robotCurrentPose()
        image = self.takeImage('217.jpg')
        self.base2chessboard_pose = self.detector.chessboardSquare(image, base2TCP_pose)
        self.__gameStandby()
        self.manipulator.goToJointState(self.standby)
        return "Detection accomplished"

    def __gameStandby(self):
        tvect = self.camera_pose.to_tvec()
        x, y = tvect[0] + 0.1333025, tvect[1]
        angle = np.arccos(x / norm(np.array([x,y]))) / np.pi * 180
        self.standby = [angle,-135,90,-70,-90,0.0]
        return None

    def __takeImagePose(self,base2chessboard_pose):
        z = (self.camera_matrix[0][0] * (-0.18)) / (400 - self.camera_matrix[0][2])
        tfmatrix = base2chessboard_pose.to_tfmatrix()
        tfmatrix[:,-1] = np.matmul(tfmatrix,np.array([0.18,0.18,-z,1]))
        tfmatrix = np.matmul(tfmatrix,inv(self.TCP2camera_pose.to_tfmatrix()))
        self.camera_pose = Trans3D.from_tfmatrix(tfmatrix)
        return self.camera_pose

    def takeImage(self,file_name):
        img = cv2.imread(file_name)
        return img

    def chessboardState(self):
        image = self.takeImage(file_name)
        board = self.detector.chessboardState(image)
        if self.board == None:
            self.board = self.__humanCheck(board)
        else:
            self.board = self.__systemCheck(board)
        return self.detector.chessboardTOFen(self.board)

    def __humanCheck(self,board):
        pass
    def __systemCheck(self,board):
        pass       
    
    def carryOutOrder(self,detail):
        square,capturing,castling = detail.split(',')
        start, end = self.squareToIndex(square[:2]), self.squareToIndex(square[2:])
        pickup_dict = {'k':0.065,'q':0.065,'b':0.04,'n':0.037,'r':0.037,'p':0.03}
        if capturing == 'yes':
            pass
        elif castling == 'yes':
            pass
        else:
            #piece = (self.chessboard[start[0]][start[1]]).lower()
            piece = 'k'
            pickup_height = pickup_dict[piece]
            if piece != 'n':
                raiseup_height = pickup_height + 0.007
            if piece == 'n':
                raiseup_height = self.raiseUpKnight(start,end) + pickup_height
            waypoints = self.pickAndPlaceWaypoints(start,end,pickup_height,raiseup_height)
            self.manipulator.goStraightToPose(waypoints)
        return 'Finished'
    
    def squareToIndex(self,square):
        alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        return (int(square[1])-1, alf_dict[square[0]])

    def raiseUpKnight(self,start,end):
        piece_height = {'k':0.105,'q':0.095,'b':0.08,'n':0.06,'r':0.06,'p':0.05,'_':0.007}
        row = [start[0], end[0]]
        col = [start[1], end[1]]
        chessboard = self.chessboard.copy()
        chessboard[start[0]][start[1]] = '_'
        passing_area = chessboard[min(row):max(row)+1,min(col):max(col)+1]
        return max([piece_height[i.lower()] for i in passing_area.reshape((passing_area.size,))])
    
    def waypoint_generator(self,square,height):
        unit_length = 0.045/2
        tfmatrix = self.base2chessboard_pose.to_tfmatrix().copy()
        tfmatrix[:,-1] = np.matmul(tfmatrix,np.array([(2*square[1]+1)*unit_length,(2*square[0]+1)*unit_length,-height,1]))
        tfmatrix_pose = Trans3D.from_tfmatrix(tfmatrix)
        return tfmatrix_pose

    def pickAndPlaceWaypoints(self,start,end,pickup_height,raiseup_height):
        pickup = self.waypoint_generator(start, pickup_height)
        start_raiseup = self.waypoint_generator(start, raiseup_height)
        end_raiseup = self.waypoint_generator(end, raiseup_height)
        dropoff = self.waypoint_generator(end, pickup_height-0.003)
        waypoints = [pickup, start_raiseup, end_raiseup, dropoff]
        return waypoints
    '''
    def pickAndPlace(self,start,end,pickup_height,raiseup_height):
        pickup_dict = {'k':0.065,'q':0.065,'b':0.04,'n':0.036,'r':0.036,'p':0.03}
        piece = (self.chessboard[start[0]][start[1]]).lower()
        pickup_height = pickup_dict[piece]
        if piece != 'n' and not capturing:
            raiseup_height = pickup_height + 0.007
        elif piece == 'n' and not capturing:
            raiseup_height = self.raiseUpKnight(start,end)

    def __criticalPoints(self,square,action,raiseup):
            alf_dict = {'H':0,'G':1,'F':2,'E':3,'D':4,'C':5,'B':6,'A':7}
            alf,num = alf_dict[sq[0]],sq[1]
            piece_dict = {'k':0.065,'q':0.065,'b':0.04,'n':0.037,'r':0.037,'p':0.03}
            pp = - (piece_dict[(self.board[alf][num]).lower()] + gripper_high)
            if action == 'pick': pass
            else : hight + 0.005
            pp_vect = np.array([(alf+1) * unit_length / 2, (num) * unit_length / 2, pp, 1])
            tfmatrix = self.base2chessboard_pose.to_tfmatrix()
            tfmatrix[:,-1] = np.matmul(tfmatrix,vect)
            return Trans3D.from_tfmatrix(tfmatrix)
    
    def __movePiece(self,start,end):
        gripper_high
        piece_dict = {'k':0.065,'q':0.065,'b':0.04,'n':0.037,'r':0.037,'p':0.03}
        pickup = - (piece_dict[(self.board[alf][num]).lower()] + gripper_high)
        dropoff = pickup + 0.005
        if no piece in between
        if piece in between
        raise up high
        drop off high 
        start = self.__squareToTfmatrix(start)
        end = self.__squareToTfmatrix(end)
        self.manipulator.pickPlace(start,end)
    '''

if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotService()
    rospy.spin()
    #cv2.imshow("img",img)
    #cv2.waitKey(0)