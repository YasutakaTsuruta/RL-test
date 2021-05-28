#!/usr/bin/env python

import  socket
from proto import ssl_vision_wrapper_pb2

class receiver():
    def __init__(self):
        multicast_if_addr='192.168.10.123'  

        multicast_group='224.5.23.2'
        multicast_port=10006

        my_addr='0.0.0.0'
        server_address=(my_addr, multicast_port)

        self.sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(server_address)

        mreq=socket.inet_aton(multicast_group)+socket.inet_aton(multicast_if_addr)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.robot = robot()
        self.ball = ball()

    def receive(self):
        while True:
            buf = ""
            # 複数のカメラからデータを受け取るため、whileループで全て受け取る
            while buf is not False:
                buf = self.sock.recv(2048)
                if buf:
                    # do something
                    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
                    packet.ParseFromString(buf)

                    if packet.HasField('detection'):
                         # ball
                        for ball in packet.detection.balls:
                            self.ball.x = ball.x
                            self.ball.y = ball.y
                        # yellow robot
                        for robot in packet.detection.robots_yellow:
                            if robot.robot_id == 7:
                                self.robot.id =robot.robot_id
                                self.robot.x = robot.x
                                self.robot.y = robot.y
                                self.robot.theta = robot.orientation

class robot():
    def __init__(self):
        self.id = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

class ball():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
