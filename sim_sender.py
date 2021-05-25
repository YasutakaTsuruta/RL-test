# coding: UTF-8

import socket
import math
import time
import datetime

from proto import grSim_Packet_pb2

class SimSender(object):
    def __init__(self):
        self._host = '127.0.0.1'
        self._port = 20011

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._MAX_KICK_SPEED = 8.0 # m/s


    def send_commands(self, vel):
        packet = grSim_Packet_pb2.grSim_Packet()
        packet.commands.timestamp = datetime.time()
        packet.commands.isteamyellow = True

        packet_command = packet.commands.robot_commands.add()

        # ロボットID
        packet_command.id = command.robot_id

        # 走行速度
        packet_command.veltangent = command.vel_surge if not math.isnan(command.vel_surge) else 0
        packet_command.velnormal = command.vel_sway if not math.isnan(command.vel_sway) else 0
        packet_command.velangular = command.vel_angular if not math.isnan(command.vel_angular) else 0

        # キック速度
        packet_command.kickspeedx = command.kick_power * self._MAX_KICK_SPEED

        # チップキック
        if command.chip_enable:
            packet_command.kickspeedz = packet_command.kickspeedx
        else:
            packet_command.kickspeedz = 0

        # ドリブラー
        if command.dribble_power > 0:
            packet_command.spinner = True
        else:
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
