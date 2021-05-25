#!/usr/bin/env python2
# coding: UTF-8
import get_ball_env
import time
env = get_ball_env.get_ball_env()
while True:
    time.sleep(3)
    env.reset()
    print("poe")
