#!/usr/bin/env python2
# coding: UTF-8
import gym
import sim_sender
import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt

from proto import grSim_Packet_pb2

class get_ball_env(gym.Env):
    def __init__(self):
        # アクションの数の設定
        # robot_x, robot_y, robot_omega
        act_high = np.array([1.0, 1.0, 1.0])
        self.action_space = gym.spaces.Box(low=-act_high, high=act_high) 

        # 状態空間の設定 
        # robot_x, robot_y, robot_theta, ball_x, ball_y 
        obs_high = np.array([5.0, 4.0, np.pi, 4.5, 3.0])
        self.observation_space = gym.spaces.Box(low=-obs_high, high=obs_high)

        self.fin_pos = np.array([0, 0.09])
        self.cycle = 1.0 / 60.0
        self.steps = 0
        self.fig = plt.figure()
        self.fig.show()
        self.sender = sim_sender.SimSender()

    def reset(self):
        # シミュレータの初期化処理
        packet = grSim_Packet_pb2.grSim_Packet()
         
        ball = packet.replacement.ball
        ball.x = 1.0
        ball.y = 0.0
        ball.vx = 1.0
        ball.vy = 0.0
         
        robot = packet.replacement.robots.add()
        robot.x = 0.0
        robot.y = 0.0
        robot.dir = 0.0
        robot.id = 7
        robot.yellowteam = True
        robot.turnon = True
        self.sender.env_reset(robot, ball)
        self.robot_pos = np.array([robot.x, robot.y, robot.dir])
        self.ball_pos = np.array([ball.x, ball.y])
        self.steps = 0
        return np.concatenate([self.robot_pos, self.ball_pos])

    def step(self, action):
        # ステップを進める処理
        self.robot_pos = self.robot_pos + 0.5 * (self.robot_vel - self.ball_vel) * self.cycle
        reward = 0.0
        done = False
        if (np.linalg.norm(self.robot_pos - self.fin_pos) < np.linalg.norm(prev_robot_pos - self.fin_pos)):
            reward = 0.05
        if (np.linalg.norm(self.robot_pos - self.fin_pos) < 0.01):
            reward = 1
        if (self.steps == 100 or np.linalg.norm(self.robot_pos - self.fin_pos) > 5.0 or (np.linalg.norm(self.robot_pos - self.fin_pos) > 1.0 and np.linalg.norm(prev_robot_pos - self.fin_pos) < np.linalg.norm(self.robot_pos))):
            done = True
        self.steps = self.steps + 1
        self.render()
        #infoの部分は使わなかったので{}を返却
        return np.concatenate([self.robot_pos, self.robot_vel, self.ball_vel]), reward, done, {}

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
