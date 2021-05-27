#!/usr/bin/env python2
# coding: UTF-8
import get_ball_env
import time
import threading
import simple_receiver
env = get_ball_env.get_ball_env()
rcv = simple_receiver.receiver()
robot = rcv.robot
ball = rcv.ball
thread = threading.Thread(target=rcv.receive)
thread.start()
while True:
    time.sleep(3)
    env.reset()
    print("ooooooooooooooooooooooooooooooooooooooooooooooooooooo")
    print(robot.id)
    print(robot.x, robot.y,  robot.theta)
    print(ball.x,  ball.y)
