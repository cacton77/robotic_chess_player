#!/usr/bin/env python
import rospy
from robotic_chess_player.srv import *
from std_msgs.msg import String
import numpy as np
from io import BytesIO as StringIO

class TaskPlanning():

    def __init__(self):
        #init ros node
        rospy.init_node("task_planning_node")
        
        #connect to chess ai service
        self.ai_service = self.init_service('chess_ai_service',ChessAI)
        #connect to robot service
        self.robot_service = self.init_service('robot_service',ChessRobotService)
        #connect to neural network service
        self.nn_service = self.init_service('board_state', ChessRobotService)
        
        #subscribe message from gui
        self.gui_sub = rospy.Subscriber('/button', String, queue_size=5, callback=self.gui_callback)
        #set up the flag for actions
        self.locate_flag = False 
        self.detect_flag = False
        self.robot_flag = False
        # publishe task planning message to gui
        self.info_pub = rospy.Publisher('/info_msg', String, queue_size=5)


    def init_service(self,service_name,service_message):
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, service_message)
        rospy.loginfo('Successfully connected to {}'.format(service_name))
        return service

    def gui_callback(self, command):
        if command.data == "locate chessboard":
            self.locate_flag = True 
        if command.data == "detect chessboard":
            self.detect_flag = True
        if command.data[:7] == "confirm":
            self.board_msg = command.data.split(';')[1]
            self.robot_flag = True
      
    def run(self):
        rospy.loginfo("task planning is running now")
        while not rospy.is_shutdown():
            if self.locate_flag:
                rospy.loginfo("locating chessboard position")
                self.locating_chessboard()
                self.locate_flag = False
            elif self.detect_flag:
                rospy.loginfo("detecting chessboard state")
                state_msg = self.detecting_chessboard()
                self.info_pub.publish('det;'+state_msg)
                self.robot_service('to standby')
                self.detect_flag = False
            elif self.robot_flag:
                self.robot_move(self.board_msg)
                self.robot_flag = False
            rospy.sleep(0.1)

    def locating_chessboard(self):
        info = self.robot_service('locate chessboard').feedback
        if info[:4] ==  'Done':
            str_square_dict = info.split(';')[1]
            try:
                resp = self.nn_service(str_square_dict)
                rospy.loginfo(resp.feedback)
                msg = 'Location Accomplished'
            except rospy.ServiceException as e:
                msg = 'Location Accomplished But Neural Network Node did not receive square dictionary'
        else:
            msg = 'Location Failed, Please remove possible noise and locate again'
        self.info_pub.publish('loc;'+msg)
        return None

    def detecting_chessboard(self):
        self.robot_service('detect chessboard state')
        return self.nn_service('state').feedback

    def robot_move(self,board_msg):
        fen = self.__board2fen(board_msg)
        move = self.ai_service(fen).command
        if move != 'Game is over':
            new_board = self.robot_service('move:'+ self.board_msg + ';' + move).feedback
            self.info_pub.publish('mov;'+ new_board)
        else:self.info_pub.publish('mov;'+ move)

    def __board2fen(self,board_msg):
        board = np.rot90(np.array([list(i) for i in board_msg.split(',')]),2)
        with StringIO() as s:
            for row in board:
                empty = 0
                for cell in row:
                    if cell != '_':
                        if empty > 0:
                            s.write(str(empty))
                            empty = 0
                        s.write(cell)
                    else:
                        empty += 1
                if empty > 0:
                    s.write(str(empty))
                s.write('/')
            # Move one position back to overwrite last '/'
            s.seek(s.tell() - 1)
            # If you do not have the additional information choose what to put
            s.write(' b KQkq - 0 1')
            return s.getvalue()

if __name__ == "__main__":
    try:
        obj = TaskPlanning()
        obj.run()
    except rospy.ROSInterruptException:
        pass