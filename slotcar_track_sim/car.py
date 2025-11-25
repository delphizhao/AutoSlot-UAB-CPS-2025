# -*- coding: utf-8 -*-

from PIL import Image, ImageTk
import math
from config import *

# car sprite
car1_img = Image.open('car1.png')


class Car:
    def __init__(self, x, y, b, img, name,
                 piecewise_curvature, piecewise_angle, piecewise_position):
        self.fi = img
        self.name = name

        # Internal State
        self.x = x
        self.y = y
        self.s = 0.0          # distance along the track [m]
        self.b = b            # heading angle [rad]
        self.v = 0.0          # longitudinal speed [m/s]
        self.img = None

        # Inputs
        self.w = 0.0          # wheel angular speed [rad/s] (not used yet)
        self.iv = 0.0         # input voltage [V]

        # Motor / drivetrain parameters (defaults, will be overwritten from sliders)
        self.R = 0.5          # motor resistance [ohm]
        self.kt = 0.8         # torque constant
        self.bemf = 0.003     # back EMF constant
        self.wra = 0.004      # wheel radius [m] (4 mm default)
        self.axisdistance = 0.07  # wheelbase [m]

        self.mass = 0.07      # car mass [kg] (70 g default)
        self.us = 0.3         # static friction coeff
        self.ud = 0.2         # dynamic friction coeff

        self.gear_ratio = 2.5  # N, from slider
        self.efficiency = 0.80 # η, from slider (0..1)

        # track functions
        self.piecewise_curvature = piecewise_curvature
        self.piecewise_position = piecewise_position
        self.piecewise_angle = piecewise_angle

    def tick(self, deltat):
        """
        Advance the car state by one time step [deltat].
        Uses a simple motor + gearbox + wheel model to update longitudinal speed.
        """

        # 1) Read parameters (already in SI units)
        V = self.iv                     # input voltage [V]
        N = self.gear_ratio             # gear ratio
        eta = self.efficiency           # drivetrain efficiency (0..1)
        r = self.wra                    # wheel radius [m]
        kt = self.kt                    # torque constant
        ke = self.bemf                  # back-EMF constant
        Rm = self.R                     # motor resistance [ohm]
        m = max(self.mass, 1e-3)        # car mass [kg], avoid division by zero

        # 2) Motor + gear + wheel: compute forward force
        #    F = (eta * N / r) * (kt / R) * ( V - ke * N * v / r )
        motor_term = V - ke * N * self.v / max(r, 1e-4)
        F_drive = eta * N / max(r, 1e-4) * (kt / Rm) * motor_term
        if F_drive < 0.0:
            F_drive = 0.0

        # 3) Longitudinal dynamics: a = F/m, v += a * dt
        a_long = F_drive / m
        self.v += a_long * deltat

        # Simple drag to avoid infinite acceleration
        drag_coeff = 0.2
        self.v -= drag_coeff * self.v * deltat
        if self.v < 0.0:
            self.v = 0.0

        # 4) Update distance along the track
        incs = self.v * deltat          # Δs [m]
        self.s += incs

        # 5) Get position and angle from piecewise functions
        self.x, self.y = self.piecewise_position.get(self.s)
        self.b = self.piecewise_angle.get(self.s)

    def draw(self, canvas):
        """Draw the car sprite on the Tkinter canvas."""
        if self.img:
            canvas.delete(self.img)

        ow, oh = self.fi.size

        screen_x, screen_y = m_to_px(canvas, self.x, self.y)

        rot_angle = self.b * 180.0 / math.pi

        img = self.fi.rotate(-90.0 + rot_angle, resample=Image.BICUBIC)
        img = img.resize((int(ow * SCALE), int(oh * SCALE)),
                         Image.Resampling.LANCZOS)

        # keep a reference to avoid garbage collection
        self.photo = ImageTk.PhotoImage(img)
        self.img = canvas.create_image(screen_x, screen_y, image=self.photo)
        # print('draw car', self.x, self.y, screen_x, screen_y)
