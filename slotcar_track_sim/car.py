# -*- coding: utf-8 -*-

from PIL import Image, ImageTk
import math
from config import *

car1_img = Image.open('car1.png')

class Car:
    def __init__(self, x, y, b, img, name, piecewise_curvature, piecewise_angle, piecewise_position):
        self.cars = []
        self.fi = img
        self.name = name

        # Internal State
        self.x = x
        self.y = y
        self.s = 0 # linear distance in mm
        self.b = b # angle in radians
        self.v = 0 # linear velocity
        self.img = None
        # Inputs
        self.w = 0 # wheel angle
        self.iv = 0 # input voltage

        # parameters
        self.R = 0.5 # internal motor resistance (ohms)
        self.kt = 0.01 # torque constant (Nm/A)
        self.bemf = 0.01 # back EMF constant (V/rad/s)
        self.wra = 0.015 # wheel radius (m)
        self.axisdistance = 0.7 # 7 cm 
        self.us = 0.3 # static friction
        self.ud = 0.2 # dynamic friction
        
        self.piecewise_curvature = piecewise_curvature
        self.piecewise_position = piecewise_position
        self.piecewise_angle = piecewise_angle

    def tick(self, deltat):
        
        # @TODO dirty hack to demo
        self.v = self.iv
        
        incs = (self.v * deltat) 
        self.s += incs 
        c = self.piecewise_curvature.get(self.s) # curvature in radians/mm
        
        self.x, self.y = self.piecewise_position.get(self.s)
        
        
        # DEMO WRONG WAY
        #self.b += c * incs 
        
        self.b = self.piecewise_angle.get(self.s)



    def draw(self, canvas):
        if self.img:
            canvas.delete(self.img)

        ow, oh = self.fi.size

        screen_x, screen_y = m_to_px(canvas, self.x, self.y)

        rot_angle = self.b * 180 / math.pi

        img = self.fi.rotate(-90 + (rot_angle ) , resample=Image.BICUBIC)
        img = img.resize((int(ow*SCALE), int(oh*SCALE)), Image.Resampling.LANCZOS)
       
        self.photo = ImageTk.PhotoImage(img)
        self.img = canvas.create_image(screen_x, screen_y, image=self.photo)

        #print('draw car', self.x, self.y, screen_x, screen_y)
