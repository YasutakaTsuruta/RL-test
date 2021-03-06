# coding: UTF-8

import socket
import math
import time
import numpy as np

from proto import grSim_Packet_pb2

class SimSender(object):
    def __init__(self):
        self._host = '127.0.0.1'
        self._port = 20011

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._MAX_KICK_SPEED = 8.0 # m/s

        self._vel = np.array([0.0, 0.0])

        self._time = time.time()


    def send_commands(self, action):
        packet = grSim_Packet_pb2.grSim_Packet()
        packet.commands.timestamp = time.time()
        packet.commands.isteamyellow = True

        packet_command = packet.commands.robot_commands.add()

        # ロボットID
        packet_command.id = 7

        # 走行速度
        cycle = time.time() - self._time
        self._time = time.time()
        print(cycle)
        self._vel[0] += 3.0 * action[0] * cycle
        self._vel[1] += 3.0 * action[1] * cycle
        packet_command.veltangent = self._vel[0]
        packet_command.velnormal = self._vel[1]
        packet_command.velangular = action[2] * np.pi
        #print('start')
        #print(packet_command.veltangent)
        #print(packet_command.velnormal)
        #print(packet_command.velangular)

        # キック速度
        packet_command.kickspeedx =  0

        # チップキック
        packet_command.kickspeedz = 0

        # ドリブラー
        packet_command.spinner = False

        # タイヤ個別に速度設定しない
        packet_command.wheelsspeed = False

        message = packet.SerializeToString()
        self._sock.sendto(message, (self._host, self._port))


    def env_reset(self, robot, ball):
        packet = grSim_Packet_pb2.grSim_Packet()

        replace_ball = packet.replacement.ball
        replace_ball.x = ball.x
        replace_ball.y = ball.y
        replace_ball.vx = ball.vx
        replace_ball.vy = ball.vy

        replace_robot = packet.replacement.robots.add()
        replace_robot.x = robot.x
        replace_robot.y = robot.y
        replace_robot.dir = robot.dir
        replace_robot.id = robot.id
        replace_robot.yellowteam = robot.yellowteam
        replace_robot.turnon = robot.turnon

        message = packet.SerializeToString()
        self._sock.sendto(message, (self._host, self._port))
