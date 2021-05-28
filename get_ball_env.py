#!/usr/bin/env python2
# coding: UTF-8
import gym
import sim_sender
import simple_receiver
import threading
import time
import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt

from proto import grSim_Packet_pb2

class get_ball_env(gym.Env):
    def __init__(self):
        # アクションの数の設定
        # robot_x, robot_y
        act_high = np.array([1.0, 1.0])
        self.action_space = gym.spaces.Box(low=-act_high, high=act_high) 

        # 状態空間の設定 
        # ball_x, ball_y 
        obs_high = np.array([5.0, 5.0])
        self.observation_space = gym.spaces.Box(low=-obs_high, high=obs_high)

        self.fin_pos = np.array([0, 0.09])
        self.cycle = 1.0 / 60.0
        self.steps = 0
        self.fig = plt.figure()
        self.fig.show()
        self.sender = sim_sender.SimSender()
        rcv = simple_receiver.receiver()
        self.robot = rcv.robot
        self.ball = rcv.ball
        thread = threading.Thread(target=rcv.receive)
        thread.start()

    def reset(self):
        # シミュレータの初期化処理
        packet = grSim_Packet_pb2.grSim_Packet()
         
        ball = packet.replacement.ball
        ball.x = 9.0 * np.random.rand() - 4.5
        ball.y = 6.0 * np.random.rand() - 3.0
        ball_vel = 6.5 * np.random.rand()
        theta = 2 * np.pi * np.random.rand()
        ball.vx = ball_vel * np.cos(theta)
        ball.vy = ball_vel * np.sin(theta)
         
        robot = packet.replacement.robots.add()
        robot.x = 9.0 * np.random.rand() - 4.5
        robot.y = 6.0 * np.random.rand() - 3.0
        robot.dir = 2 * np.pi * np.random.rand()
        robot.id = 7
        robot.yellowteam = True
        robot.turnon = True
        self.sender.env_reset(robot, ball)
        time.sleep(0.2)
        self.robot_pos = np.array([robot.x * 1000.0, robot.y * 1000.0, robot.dir])
        self.ball_pos = np.array([ball.x * 1000.0, ball.y * 1000.0])
        self.steps = 0
        return np.concatenate([self.robot_pos, self.ball_pos])

    def step(self, action, omega):
        # ステップを進める処理
        action = np.array([action[0], action[1], omega])
        self.sender.send_commands(action)
        reward = 0.0
        done = False
        self.steps = self.steps + 1
        #self.render()
        #infoの部分は使わなかったので{}を返却
        #return np.concatenate([self.robot_pos, self.robot_vel, self.ball_vel]), reward, done, {}

    def render(self):
        #self.fig.show()
        ax = self.fig.add_subplot(111)
        robot = patches.Circle(xy=(self.robot_pos[0], self.robot_pos[1]), radius=0.18, fc='b')
        ball = patches.Circle(xy=(0, 0), radius=0.04, fc='orange')
        ax.add_patch(robot)
        ax.add_patch(ball)
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_aspect('equal')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.fig.clear()
