# -*- coding: utf-8 -*-

import math
from PIL import Image, ImageTk
from config import *  # SCALE, m_to_px, etc.

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
        track_length,
    ):
        self.fi = img
        self.name = name

        # ===== State =====
        self.x = x                  # world x [m]
        self.y = y                  # world y [m]
        self.s = 0.0                # distance along the track [m]
        self.b = b                  # heading angle [rad]
        self.v = 0.0                # longitudinal speed [m/s]
        self.v_lat = 0.0            # lateral speed [m/s]
        self.lat_offset = 0.0       # lateral offset from lane center [m]
        self.alpha = 0.0            # slip angle [rad]
        self.img = None
        self.photo = None
        self.derailed = False       # kept for completeness, not used to freeze

        # ===== Inputs =====
        self.iv = 0.0               # input voltage [V]

        # ===== Parameters (defaults) =====
        self.R = 0.5                # (kept for completeness, not really used)
        self.kt = 0.8
        self.bemf = 0.003
        self.wra = 0.004            # wheel radius [m] (4 mm)
        self.axisdistance = 0.07    # wheelbase [m]

        self.mass = 0.07            # car mass [kg] (70 g)
        self.us = 0.3               # static friction coefficient
        self.ud = 0.2               # dynamic friction coefficient

        self.gear_ratio = 2.5       # gear ratio N
        self.efficiency = 0.80      # geartrain efficiency (0..1)

        # Magnet parameter from slider (0..50 MGOe)
        self.mag_param = 0.0

        # Track functions and length
        self.piecewise_curvature = piecewise_curvature   # s -> curvature c [1/m]
        self.piecewise_position = piecewise_position     # s -> (x, y)
        self.piecewise_angle = piecewise_angle           # s -> track tangent angle
        self.track_length = max(track_length, 1e-6)

    # ==========================================================
    #   Time step: simple motor model + cornering + slip
    # ==========================================================
    def tick(self, deltat: float) -> None:
        m = max(self.mass, 1e-3)
        g = 9.81

        # ---------- 1) Longitudinal dynamics (simplified) ----------
        V = max(self.iv, 0.0)
        V = min(V, 12.0)

        a_max = 6.0  # m/s^2, tunable
        a_long = a_max * (V / 12.0)

        # Quadratic drag to get a reasonable terminal speed
        drag_coeff = 0.4
        drag = drag_coeff * self.v * abs(self.v)

        # dv/dt = a_drive - drag/m
        self.v += (a_long - drag / m) * deltat
        if self.v < 0.0:
            self.v = 0.0

        # Optional top speed clamp (safety)
        v_top = 15.0  # m/s
        if self.v > v_top:
            self.v = v_top

        # ---------- 2) Advance along the track center line ----------
        incs = self.v * deltat
        self.s = (self.s + incs) % self.track_length  # wrap around track

        cx, cy = self.piecewise_position.get(self.s)
        track_angle = self.piecewise_angle.get(self.s)

        # Local curvature (1/m); centripetal demand Fc = m v^2 |c|
        c = self.piecewise_curvature.get(self.s)
        abs_c = abs(c)

        if abs_c < 1e-6 or self.v <= 0.0:
            Fc_demand = 0.0
        else:
            Fc_demand = m * self.v * self.v * abs_c

        # ---------- 3) Available friction (including magnet downforce) ----------
        k_mag = 2.0  # stronger effect of the magnet slider
        N_r = m * g * (1.0 + k_mag * self.mag_param / 50.0)
        Fr_max_static = self.us * N_r
        Fr_max_dynamic = self.ud * N_r

        # ---------- 4) Slip / no-slip logic ----------
        if (
            Fc_demand <= Fr_max_static
            or abs_c < 1e-6
            or self.v <= 0.0
        ):
            # Inside static friction: car sticks to the slot
            # But gently decay any residual lateral motion.
            self.v_lat *= 0.5
            self.lat_offset *= 0.9
        else:
            # Static friction exceeded -> car starts to slide outward
            F_lat = max(Fc_demand - Fr_max_dynamic, 0.0)
            a_lat = F_lat / m
            self.v_lat += a_lat * deltat

        # Limit lateral speed
        max_v_lat = 10.0  # m/s
        if self.v_lat > max_v_lat:
            self.v_lat = max_v_lat
        elif self.v_lat < -max_v_lat:
            self.v_lat = -max_v_lat

        # ---------- 5) Integrate lateral offset & clamp to lane ----------
        self.lat_offset += self.v_lat * deltat

        max_offset = 0.04  # 4 cm from lane center
        if abs(self.lat_offset) > max_offset:
            # Clamp to boundary, but do not freeze the car
            self.lat_offset = math.copysign(max_offset, self.lat_offset)
            self.v_lat = 0.0

        # Slip angle for drawing
        self.alpha = math.atan2(self.v_lat, self.v + 1e-4)

        # Local normal direction (pointing left from tangent)
        nx = -math.sin(track_angle)
        ny = math.cos(track_angle)

        # World position = center line + lateral offset
        self.x = cx + self.lat_offset * nx
        self.y = cy + self.lat_offset * ny

        # Car heading = track tangent + slip angle
        self.b = track_angle + self.alpha

    # ==========================================================
    #   Drawing on Tkinter canvas
    # ==========================================================
    def draw(self, canvas) -> None:
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
