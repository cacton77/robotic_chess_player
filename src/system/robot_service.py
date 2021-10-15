#!/usr/bin/env python
import os
import rospy
from robotic_chess_player.srv import ChessRobotService,ChessRobotServiceResponse
import numpy as np
from numpy.linalg import norm
import rospkg
import sys
r = rospkg.RosPack()
path_1 = r.get_path('robotic_chess_player') + '/src/motion'
sys.path.append(path_1)
from motion_planning import *
path_2 = r.get_path('robotic_chess_player') + '/src/vision'
sys.path.append(path_2)
from visual_detector import *
path_3 = r.get_path('robotic_chess_player') + '/src/include'
sys.path.append(path_3)
from transformation import Trans3D
from avt_camera import *

class RobotServer:
    
    def __init__(self):

        self.server = rospy.Service('robot_service',ChessRobotService,self.serviceHandler)
        self.manipulator = MotionPlanner()
        self.board = None
        self.standby = (90,-135,90,-70,-90,0.0)
        self.detector = VisualDetector()
        self.camera = AvtCamera()
        self.EDGE = float(rospy.get_param('/parameter/edge'))

    def serviceHandler(self,msg):
        rospy.loginfo("Request: {}".format(msg.request))
        if msg.request == "to standby":
            self.manipulator.moveRobotJoint([[i for i in self.standby]])
            return ChessRobotServiceResponse('Robot arrive standby position') 
        
        elif msg.request == 'locate chessboard':
            feedback = self.locate_chessboard()
            return ChessRobotServiceResponse(feedback)
        
        elif msg.request[:2] == 'to':
            detail = msg.request.split(':')
            self.to_square(detail[1])
            return ChessRobotServiceResponse('Done')

        elif msg.request == 'detect chessboard state':
            self.manipulator.moveRobot([self.base2TCP_pose])
            return ChessRobotServiceResponse('Robot arrive taking image position')

        elif msg.request[:4] == 'move':
            detail = msg.request.split(':')[1]
            board_msg, move = detail.split(';')
            self.board = np.array([list(i) for i in board_msg.split(',')])
            self.carryOutOrder(move)
            new_board = self.__boardToMsg()
            return ChessRobotServiceResponse(new_board)

        elif msg.request[:4] == 'auto':
            detail = msg.request.split(';')[1]
            self.collectData(detail)
            return ChessRobotServiceResponse('Done')

    def locate_chessboard(self):
        try:
            self.manipulator.moveRobotJoint([[i for i in self.standby]])
            rospy.sleep(1)
            base2TCP_pose = self.manipulator.currentRobotPose()
            closer_pose = self.detector.closer_view_pose(base2TCP_pose)
            self.manipulator.moveRobot([closer_pose])
            rospy.sleep(1)
            base2TCP_pose = self.manipulator.currentRobotPose()
            self.base2chessboard_pose, self.base2TCP_pose, self.spot = self.detector.position_pose(base2TCP_pose)
            self.manipulator.moveRobot([self.base2TCP_pose])
            rospy.sleep(1)
            str_square_dict = self.detector.generate_square_dict()
            self.standby = self.__gameStandby()
            self.manipulator.moveRobotJoint([[i for i in self.standby]])
            self.capture_piece = 0
            return 'Done;'+ str_square_dict
        except:
            return 'Fail'

    def __gameStandby(self):
        tvect = self.manipulator.currentRobotPose().to_tvec()
        x = np.array([tvect[0] - 0.132, tvect[1]])
        angle = np.arccos(-(x / norm(x))[0]) / np.pi * 180
        return (float(angle), -135, 90, -70, -90, 0.0)   

    def to_square(self,square):
        square = square.split(',')
        square_pose = self.__squarePose(square[0])
        print('square translation: ',square_pose.to_tvec())
        if len(square) == 1: 
            self.manipulator.moveRobot([square_pose])
        elif len(square) ==  2:
            pickup_height = float(square[1])
            pickup_pose = square_pose * Trans3D.from_tvec(np.array([0,0,-pickup_height]))
            self.manipulator.moveRobot([pickup_pose])
        return 'Done'
        
    def __squarePose(self,square):
        "x : alf , y : num"
        if square == 'spot':
            self.capture_piece += 1
            return self.spot
        else:
            unit_length = self.EDGE/2
            alf_dict = {'h':1,'g':2,'f':3,'e':4,'d':5,'c':6,'b':7,'a':8}
            x, y = (2*alf_dict[square[0]]-1) * unit_length, (2*(int(square[1:]))-1) * unit_length
            point = Trans3D.from_tvec(np.array([x, y, 0]))
            square_pose = (self.base2chessboard_pose * point).to_tfmatrix()
            return Trans3D.from_tfmatrix(square_pose)

    def carryOutOrder(self,order):
        #decode order
        detail = order.split(',')
        if len(detail) == 4:
            step,capturing,castling = detail[:3] 
            start,end = step[:2],step[2:]
            piece,raiseup_height = self.__raiseUpHeight(start,end,'move')
            self.pick_and_place(piece,start,end,raiseup_height)
        else:
            step,capturing,castling = detail
            start,end = step[:2],step[2:]
            if capturing == 'yes':
                if len(end) == 3:
                    end = end[:2]
                piece, raiseup_height = self.__raiseUpHeight(end,'spot','capturing')
                self.pick_and_place(piece,end,'spot',raiseup_height)
                piece,raiseup_height = self.__raiseUpHeight(start,end,'move')
                self.pick_and_place(piece,start,end,raiseup_height)
            elif castling == 'yes':
                if step == 'e8g8':
                    piece,raiseup_height = self.__raiseUpHeight('h8','f8','move')
                    self.pick_and_place(piece,'h8','f8',raiseup_height)
                    self.__updateState(piece,'h8','f8')
                else:
                    piece,raiseup_height = self.__raiseUpHeight('a8','d8','move')
                    self.pick_and_place(piece,'a8','d8',raiseup_height)
                    self.__updateState(piece,'a8','d8')
                piece,raiseup_height = self.__raiseUpHeight(start,end,'castling')
                self.pick_and_place(piece,start,end,raiseup_height)
            else:
                piece,raiseup_height = self.__raiseUpHeight(start,end,'move')
                self.pick_and_place(piece,start,end,raiseup_height)
            self.manipulator.moveRobotJoint([[90,-135,90,-70,-90,0.0]])
        self.__updateState(piece,start,end)

    def __updateState(self,piece,start,end):
        col = {'a':7,'b':6,'c':5,'d':4,'e':3,'f':2,'g':1,'h':0}
        self.board[int(end[1])-1,col[end[0]]] = piece
        self.board[int(start[1])-1,col[start[0]]] = '_'

    def __raiseUpHeight(self,start,end,action):
        def square_to_index(square):
            alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
            return (int(square[1])-1, alf_dict[square[0]])
        start = square_to_index(start)
        chessboard = self.board.copy()
        piece = chessboard[start[0]][start[1]]
        if action == 'capturing':
            return piece, 0.12
        if action == 'castling':
            return piece, 0.055
        else:
            piece_height = {'k':0.10,'q':0.085,'b':0.07,'n':0.055,'r':0.055,'p':0.044,'_':0.01}
            if piece.lower() == 'n':
                end = square_to_index(end)
                row = [start[0], end[0]]
                col = [start[1], end[1]]
                chessboard[start[0]][start[1]] = '_'
                passing_area = chessboard[min(row):max(row)+1,min(col):max(col)+1]
                return piece, max([piece_height[i.lower()] for i in passing_area.reshape((passing_area.size,))])
            else: return piece, 0.02

    def pick_and_place(self,piece, start, end, raiseup_height):
        s_pose, e_pose = self.__squarePose(start), self.__squarePose(end)
        ab_s_pose, ab_e_pose = self.__aboveSquarePose(s_pose), self.__aboveSquarePose(e_pose)
        pickup_pose, dropoff_pose, pickup_height = self.__pickDropPose(piece,s_pose,e_pose)
        raiseup_height = pickup_height + raiseup_height
        ra_s_pose,ra_e_pose = self.__raiseUpPose(raiseup_height,s_pose,e_pose)
        waypoints = [ab_s_pose, pickup_pose, 0,ra_s_pose,ra_e_pose,dropoff_pose, 1, ab_e_pose,2]
        self.manipulator.moveRobotWaypoints(waypoints)
        return None

    def __aboveSquarePose(self, square_pose):
            return square_pose * Trans3D.from_tvec(np.array([0,0,-0.12]))

    def __pickDropPose(self,piece,start_pose,end_pose):
            pickup_dict = {'k':0.06,'q':0.0585,'b':0.04,'n':0.0375,'r':0.032,'p':0.024}
            pickup_height = pickup_dict[piece.lower()]
            pickup_pose = start_pose * Trans3D.from_tvec(np.array([0,0,-pickup_height]))
            dropoff_pose = end_pose * Trans3D.from_tvec(np.array([0,0,-pickup_height]))
            return pickup_pose, dropoff_pose, pickup_height
    
    def __raiseUpPose(self,raiseup_height,start_pose,end_pose):
        ra_s_pose = start_pose * Trans3D.from_tvec(np.array([0,0,-raiseup_height]))
        ra_e_pose = end_pose * Trans3D.from_tvec(np.array([0,0,-raiseup_height]))
        return ra_s_pose,ra_e_pose
    
    def __boardToMsg(self):
        board_msg = str()
        for row in self.board:
            row_msg = str()
            for i in row:
                row_msg += i
            board_msg += row_msg + ','
        return board_msg[:-1]

    def collectData(self,piece):
        parent_dir = os.getcwd()
        alf_list = ['h','g','f','e','d','c','b','a']
        path = os.path.join(parent_dir, piece)
        os.mkdir(path)
        count = 200
        if piece.lower() != 'k':
            for word in alf_list:
                for num in range(1,9,2):
                    if word == 'h' and num == 1:
                        self.manipulator.moveRobot([self.base2TCP_pose])
                        rospy.sleep(0.5)
                        self.camera.trigger_image()
                        sq1, sq2 = word + str(num), word + str(num+1)
                        count = self.detector.crop_image(self.camera.lastest_img,[sq1,sq2],path,count)
                    elif num == 1:
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pick_and_place(piece,pre_sq1,sq1,0.02)
                        sq2, pre_sq2 = word+str(num+1), sq2
                        self.pick_and_place(piece,pre_sq2,sq2,0.02)
                        self.manipulator.moveRobot([self.base2TCP_pose])
                        rospy.sleep(0.5)
                        self.camera.trigger_image()
                        count = self.detector.crop_image(self.camera.lastest_img,[sq1,sq2],path,count)
                    else:
                        sq2, pre_sq2 = word+str(num+1), sq2
                        self.pick_and_place(piece,pre_sq2,sq2,0.02)
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pick_and_place(piece,pre_sq1,sq1,0.02)
                        self.manipulator.moveRobot([self.base2TCP_pose])
                        rospy.sleep(0.5)
                        self.camera.trigger_image()
                        count = self.detector.crop_image(self.camera.lastest_img,[sq1,sq2],path,count)
        else:
            self.manipulator.moveRobot([self.base2TCP_pose])
            rospy.sleep(0.5)
            self.camera.trigger_image()
            sq1 = 'h1'
            count = self.detector.crop_image(self.camera.lastest_img,[sq1],path,count)
            for word in alf_list:
                for num in range(1,9):
                    if word == 'h' and num == 1:continue
                    else:
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pick_and_place(piece,pre_sq1,sq1,0.02)
                        self.manipulator.moveRobot([self.base2TCP_pose])
                        rospy.sleep(0.5)
                        self.camera.trigger_image()
                        count = self.detector.crop_image(self.camera.lastest_img,[sq1],path,count)
        return 'Finished'
        
if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotServer()
    rospy.spin()
