#!/usr/bin/env python

from collections import deque
import random
import cv2
from PIL import Image, ImageSequence
import numpy



N_N = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/N_N.gif"
N_B = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/N_B.gif"

N_L_1 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/N_L_1.gif"
N_L_2 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/N_L_2.gif"
N_L_3 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/N_L_3.gif"
N_R_1 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/N_R_1.gif"
N_R_2 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/N_R_2.gif"
N_R_3 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/N_R_3.gif"

E_W_1 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/E_W_1.gif"
E_W_2 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/E_W_2.gif"
E_W_3 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/E_W_3.gif"
E_G_1 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/E_G_1.gif"
E_G_2 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/E_G_2.gif"
E_G_3 = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/E_G_3.gif"

OP = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/OP.gif"
ED = "/home/deli_robot/Auto_Deli_Robot/src/emoji_screen/gif/ED.gif"


def show(path):
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


class emoji:

    def __init__(self):
        screen = cv2.namedWindow("emo", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("emo", 1920, 400)
        cv2.setWindowProperty("emo", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.order_queue = deque()
        self.flag = 0

    def error(self):
        show(E_W_1)
        while True:
            show(E_W_2)
            if len(self.order_queue) != 0:
                temp_flag = self.order_queue.popleft()
                if temp_flag != 3:
                    show(E_W_3)
                    return temp_flag

    def noGPS(self):
        show(E_G_1)
        while True:
            show(E_G_2)
            if len(self.order_queue) != 0:
                temp_flag = self.order_queue.popleft()
                if temp_flag != 4:
                    show(E_G_3)
                    return temp_flag

    def add_order(self, flag_num):
        self.order_queue.append(flag_num)

    def check_order(self):
        if len(self.order_queue) == 0:
            return -1
        temp = self.order_queue.pop()
        self.order_queue.append(temp)
        return temp

    def run(self):
        show(OP)
        while True:
            if self.flag != 0:
                if self.flag == 3:
                    print("error")
                    self.flag = self.error()
                elif self.flag == 4:
                    print("noGPS")
                    self.flag = self.noGPS()
                elif self.flag == 5:
                    print("Program end")
                    show(ED)
                    cv2.destroyAllWindows()
                    break
            else:
                normal()
                if len(self.order_queue) != 0:
                    temp_flag = self.order_queue.popleft()
                    self.flag = temp_flag


