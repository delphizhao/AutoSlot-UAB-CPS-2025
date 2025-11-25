# -*- coding: utf-8 -*-

from PIL import Image, ImageTk
import math
from config import *

# Car sprite
car1_img = Image.open("car1.png")


class Car:
    def __init__(
        self,
        x,
        y,
        b,
        img,
        name,
        piecewise_curvature,
        piecewise_angle,
        piecewise_position,
    ):
        self.fi = img
        self.name = name

        # --- State variables ---
        self.x = x                  # world x [m]
        self.y = y                  # world y [m]
        self.s = 0.0                # distance along track centerline [m]
        self.b = b                  # heading angle [rad]
        self.v = 0.0                # longitudinal speed along track [m/s]
        self.v_lat = 0.0            # lateral speed (sideways) [m/s]
        self.lat_offset = 0.0       # lateral offset from lane center [m]
        self.alpha = 0.0            # slip angle [rad]
        self.img = None

        # --- Inputs ---
        self.w = 0.0                # wheel angular speed [rad/s] (unused)
        self.iv = 0.0               # input voltage [V]

        # --- Parameters (defaults) ---
        self.R = 0.5                # motor resistance [ohm]
        self.kt = 0.8               # torque constant
        self.bemf = 0.003           # back EMF constant
        self.wra = 0.004            # wheel radius [m] (4 mm)
        self.axisdistance = 0.07    # wheelbase [m]

        self.mass = 0.07            # mass [kg]
        self.us = 0.3               # static friction coefficient
        self.ud = 0.2               # dynamic friction coefficient

        self.gear_ratio = 2.5       # gearbox ratio N
        self.efficiency = 0.80      # drivetrain efficiency (0..1)

        # Magnetic downforce parameter, driven by "Magnet Max Energy Product" slider
        self.mag_param = 0.0        # 0..50 (dimensionless)

        # Track geometry functions
        self.piecewise_curvature = piecewise_curvature  # s -> curvature
        self.piecewise_position = piecewise_position    # s -> (x, y)
        self.piecewise_angle = piecewise_angle          # s -> heading angle of track

    # ------------------------------------------------------------------
    #  Time update: motor + drivetrain + lateral slip model
    # ------------------------------------------------------------------
    def tick(self, deltat: float):
        # 1) Motor + drivetrain: compute forward driving force
        V = self.iv
        N = self.gear_ratio
        eta = self.efficiency
        r = max(self.wra, 1e-4)
        kt = self.kt
        ke = self.bemf
        Rm = max(self.R, 1e-4)
        m = max(self.mass, 1e-3)

        motor_term = V - ke * N * self.v / r
        F_drive = eta * N / r * (kt / Rm) * motor_term
        if F_drive < 0.0:
            F_drive = 0.0

        # 2) Longitudinal dynamics
        a_long = F_drive / m
        self.v += a_long * deltat

        # Simple aerodynamic/rolling drag to avoid unbounded growth
        drag_coeff = 0.2
        self.v -= drag_coeff * self.v * deltat
        if self.v < 0.0:
            self.v = 0.0

        # 3) Update distance along centerline
        incs = self.v * deltat
        self.s += incs

        # Centerline position and tangent angle at the new arclength
        cx, cy = self.piecewise_position.get(self.s)
        track_angle = self.piecewise_angle.get(self.s)

        # Local curvature c(s) [rad/m]; |c| ~ 1/R
        c = self.piecewise_curvature.get(self.s)
        g = 9.81

        # 4) Required lateral (centripetal) force
        if abs(c) < 1e-6 or self.v <= 0.0:
            F_c_demand = 0.0
        else:
            R_curve = 1.0 / abs(c)
            F_c_demand = m * self.v * self.v / R_curve

        # 5) Maximum available friction (including magnetic downforce)
        #    Normal load: N_r = m g (1 + k * mag_param / 50)
        k_mag = 1.0
        N_r = m * g * (1.0 + k_mag * self.mag_param / 50.0)
        F_fric_max_static = self.us * N_r

        # 6) Check if we exceed static friction (start to slide)
        if F_c_demand <= F_fric_max_static:
            # No gross sliding: just small lateral damping back to center
            lat_damp = 4.0
            self.v_lat -= lat_damp * self.v_lat * deltat
        else:
            # Sliding: friction limited by dynamic coefficient
            F_fric_dynamic = self.ud * N_r
            # Extra force that cannot be balanced becomes outward lateral accel
            F_excess = F_c_demand - F_fric_dynamic
            a_lat = F_excess / m
            self.v_lat += a_lat * deltat

        # Extra damping on straights to bring the car back to center
        if abs(c) < 1e-6:
            lat_damp_straight = 6.0
            self.v_lat -= lat_damp_straight * self.v_lat * deltat

        # 7) Integrate lateral offset and limit it
        self.lat_offset += self.v_lat * deltat

        # Clamp lateral offset so the car visibly "slides to the edge"
        # instead of disappearing from the track.
        max_offset = 0.04  # 4 cm
        if self.lat_offset > max_offset:
            self.lat_offset = max_offset
            self.v_lat = 0.0
        elif self.lat_offset < -max_offset:
            self.lat_offset = -max_offset
            self.v_lat = 0.0

        # Slip angle = angle between velocity vector and car longitudinal axis
        self.alpha = math.atan2(self.v_lat, self.v + 1e-4)

        # Local normal direction: rotate track tangent by +90 degrees
        nx = -math.sin(track_angle)
        ny = math.cos(track_angle)

        # Final world position with lateral offset applied
        self.x = cx + self.lat_offset * nx
        self.y = cy + self.lat_offset * ny

        # Car heading = track tangent + slip angle
        self.b = track_angle + self.alpha

    # ------------------------------------------------------------------
    #  Drawing
    # ------------------------------------------------------------------
    def draw(self, canvas):
        if self.img:
            canvas.delete(self.img)

        ow, oh = self.fi.size
        screen_x, screen_y = m_to_px(canvas, self.x, self.y)
        rot_angle = self.b * 180.0 / math.pi

        img = self.fi.rotate(-90.0 + rot_angle, resample=Image.BICUBIC)
        img = img.resize(
            (int(ow * SCALE), int(oh * SCALE)),
            Image.Resampling.LANCZOS,
        )

        self.photo = ImageTk.PhotoImage(img)
        self.img = canvas.create_image(screen_x, screen_y, image=self.photo)
