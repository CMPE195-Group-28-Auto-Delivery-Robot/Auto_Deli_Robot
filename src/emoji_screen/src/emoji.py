#!/usr/bin/env python

import os
import time
import random
import cv2
#import pyglet
from PIL import Image, ImageSequence
import numpy

import termios
import atexit
from select import select



N_N = "gif/N_N.gif"
N_B = "gif/N_B.gif"

N_L_1 = "gif/N_L_1.gif"
N_L_2 = "gif/N_L_2.gif"
N_L_3 = "gif/N_L_3.gif"
N_R_1 = "gif/N_R_1.gif"
N_R_2 = "gif/N_R_2.gif"
N_R_3 = "gif/N_R_3.gif"

E_W_1 = "gif/E_W_1.gif"
E_W_2 = "gif/E_W_2.gif"
E_W_3 = "gif/E_W_3.gif"
E_G_1 = "gif/E_G_1.gif"
E_G_2 = "gif/E_G_2.gif"
E_G_3 = "gif/E_G_3.gif"

OP = "gif/OP.gif"
ED = "gif/ED.gif"

screen = cv2.namedWindow("emo", cv2.WINDOW_NORMAL)
cv2.resizeWindow("emo", 1920, 400)
cv2.setWindowProperty("emo", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
order_queue = []
flag = 0


def show(path):
    global screen
    for frame in ImageSequence.Iterator(Image.open(path)):
        frame = frame.convert('RGB')
        cv2_frame = numpy.array(frame)
        show_frame = cv2.cvtColor(cv2_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("emo", show_frame)
        cv2.waitKey(15)


def normal():
    num = random.randint(1, 16)
    if num <= 6:
        show(N_N)
    elif num <= 10:
        show(N_B)
    elif num == 11:
        show(N_L_1)
    elif num == 12:
        show(N_L_2)
    elif num == 13:
        show(N_L_3)
    elif num == 14:
        show(N_R_1)
    elif num == 15:
        show(N_R_2)
    elif num == 16:
        show(N_R_3)


def error():
    global order_queue
    show(E_W_1)
    while True:
        show(E_W_2)
        temp_flag = order_queue.pop()
        if temp_flag:
            show(E_W_3)
            return temp_flag


def noGPS():
    global order_queue
    show(E_G_1)
    while True:
        show(E_G_2)
        temp_flag = order_queue.pop()
        if temp_flag:
            show(E_G_3)
            return temp_flag


def add_order(flag_num):
    global order_queue
    order_queue.append(flag_num)


def check_order():
    global flag
    return flag


def run():
    global order_queue
    global flag
    show(OP)
    while True:
        if flag != 0:
            if flag == 3:
                print("error")
                flag = error()
            elif flag == 4:
                print("noGPS")
                flag = noGPS()
            elif flag == 5:
                print("Program end")
                show(ED)
                break
        else:
            normal()
            temp_flag = order_queue.pop()
            if temp_flag:
                flag = temp_flag


