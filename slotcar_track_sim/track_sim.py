# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk
import numpy as np

from track import *   # C8205Track, C8204Track, piecewise functions, m_to_px
from config import *
from car import Car, car1_img

# Simulation time step [s]
deltat = 0.05  # 50 ms

# Window size
sw = 1600
sh = 800


class App:
    # label, min, max, resolution, unit, internal_name
    param_definitions = [
        ("Voltage", 0.0, 12.0, 0.1, "V", "voltage"),
        ("Magnet Max Energy Product", 0.0, 50.0, 0.5, "MGOe", "max_energy"),
        ("Mass", 70.0, 200.0, 1.0, "g", "mass"),
        ("Static Friction", 0.0, 2.0, 0.01, "-", "static_f"),
        ("Dynamic Friction", 0.0, 2.0, 0.01, "-", "dynamic_f"),
        ("Wheel Radius", 4.0, 10.0, 0.1, "mm", "wheel_r"),
        ("Torque Constant", 0.8, 2.0, 0.01, "-", "torque_c"),
        ("Back EMF Constant", 1.0, 5.0, 0.01, "-", "back_emf_c"),
        ("Back EMF", 0.003, 0.007, 0.0001, "-", "back_emf"),
        ("Gear Ratio", 2.5, 4.0, 0.1, "-", "gear_ratio"),
        ("Geartrain Efficiency", 80.0, 95.0, 1.0, "%", "efficiency"),
    ]

    def __init__(self, parent: tk.Tk):
        self.parent = parent

        self.parent.title("Simulation")
        self.parent.geometry(f"{sw}x{sh}")

        self.parameters = {}
        self.cars = []

        # Layout: left control panel, right canvas
        self.parent.grid_columnconfigure(0, weight=3)
        self.parent.grid_columnconfigure(1, weight=7)
        self.parent.grid_rowconfigure(0, weight=1)

        self.setup_control_panel()

        self.canvas = tk.Canvas(parent, bg="white")
        self.canvas.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # Build track after layout is ready (canvas size not 1x1 anymore)
        self.parent.after(50, self.init_circuit)

        # Start redraw loop
        self.parent.after(int(deltat * 1000), self.redraw)

    # ----------------------------------------------------------
    #   Control panel / sliders
    # ----------------------------------------------------------
    def setup_control_panel(self) -> None:
        self.control_frame = ttk.Frame(
            self.parent,
            padding="10 10 10 10",
            relief=tk.RAISED
        )
        self.control_frame.grid(row=0, column=0, sticky="nsew")
        self.control_frame.grid_rowconfigure(0, weight=0)
        self.control_frame.grid_rowconfigure(1, weight=1)

        ttk.Label(
            self.control_frame,
            text="System Parameters",
            font=("Arial", 16, "bold")
        ).grid(row=0, column=0, columnspan=3, pady=(0, 15), sticky="w")

        self.sliders_container = ttk.Frame(self.control_frame)
        self.sliders_container.grid(row=1, column=0, columnspan=3, sticky="nsew")

        self.create_sliders(self.sliders_container)

    def create_sliders(self, parent: ttk.Frame) -> None:
        parent.grid_columnconfigure(0, weight=1)  # label
        parent.grid_columnconfigure(1, weight=3)  # slider
        parent.grid_columnconfigure(2, weight=1)  # value label

        row_index = 0
        for label_text, min_val, max_val, resolution, unit, var_name in self.param_definitions:
            ttk.Label(parent, text=f"{label_text}:").grid(
                row=row_index, column=0, padx=5, pady=5, sticky="w"
            )

            # >>> 所有参数默认都用最小值 <<<
            initial_value = min_val
            self.parameters[var_name] = initial_value

            value_label = ttk.Label(
                parent,
                text=f"{initial_value:.4f} {unit}",
                width=12
            )
            value_label.grid(row=row_index, column=2, padx=5, pady=5, sticky="e")

            def update_value_wrapper(name, unit, label):
                return lambda val: self.update_value(name, unit, label, val)

            slider = ttk.Scale(
                parent,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                command=update_value_wrapper(var_name, unit, value_label)
            )
            slider.set(initial_value)
            slider.grid(row=row_index, column=1, padx=5, pady=5, sticky="ew")

            row_index += 1

    def update_value(self, var_name: str, unit: str,
                     value_label: ttk.Label, value: str) -> None:
        new_value = float(value)
        self.parameters[var_name] = new_value

        # Formatting for the numeric label
        if var_name == "back_emf":
            formatted_value = f"{new_value:.4f}"
        elif new_value == round(new_value):
            formatted_value = f"{int(new_value)}"
        else:
            formatted_value = (
                f"{new_value:.1f}"
                if (new_value % 1) == 0.0
                else f"{new_value:.2f}"
            )

        value_label.config(text=f"{formatted_value} {unit}")

        # Push to car if it already exists
        if self.cars:
            car = self.cars[0]
            if var_name == "voltage":
                car.iv = new_value
            elif var_name == "mass":
                car.mass = new_value / 1000.0       # g -> kg
            elif var_name == "static_f":
                car.us = new_value
            elif var_name == "dynamic_f":
                car.ud = new_value
            elif var_name == "wheel_r":
                car.wra = new_value / 1000.0        # mm -> m
            elif var_name == "torque_c":
                car.kt = new_value
            elif var_name == "back_emf":
                car.bemf = new_value
            elif var_name == "gear_ratio":
                car.gear_ratio = new_value
            elif var_name == "efficiency":
                car.efficiency = new_value / 100.0  # % -> [0..1]
            elif var_name == "max_energy":
                car.mag_param = new_value

    # ----------------------------------------------------------
    #   Track and car creation
    # ----------------------------------------------------------
    def init_circuit(self) -> None:
        piecewise_curvature = CurvaturePiecewiseFunction()
        piecewise_position = PositionPiecewiseFunction()
        piecewise_angle = AnglePiecewiseFunction()

        lane_idx = 0
        initial_x, initial_y = -100 / 1000, -350 / 1000

        if lane_idx == 0:
            lane_y = (LANE_SPACING / 2 + LANE_SPACING) / 1000.0
        elif lane_idx == 1:
            lane_y = (LANE_SPACING / 2) / 1000.0
        else:
            lane_y = 0.0

        draw_tarmac = True
        draw_parametric_curve = True

        x, y, a = initial_x, initial_y, 0.0

        # Straight + curves (oval) as in the original code
        t = C8205Track(x, y, a)
        if draw_tarmac:
            t.draw(self.canvas)
        piecewise_curvature.appendTrack(t, lane_idx)
        piecewise_position.appendTrack(t, lane_idx)
        piecewise_angle.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        for _ in range(4):
            t = C8204Track(x, y, a, "L")
            if draw_tarmac:
                t.draw(self.canvas)
            piecewise_curvature.appendTrack(t, lane_idx)
            piecewise_position.appendTrack(t, lane_idx)
            piecewise_angle.appendTrack(t, lane_idx)
            x, y, a = t.getNext()

        t = C8205Track(x, y, a)
        if draw_tarmac:
            t.draw(self.canvas)
        piecewise_curvature.appendTrack(t, lane_idx)
        piecewise_position.appendTrack(t, lane_idx)
        piecewise_angle.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        for _ in range(4):
            t = C8204Track(x, y, a, "L")
            if draw_tarmac:
                t.draw(self.canvas)
            piecewise_curvature.appendTrack(t, lane_idx)
            piecewise_position.appendTrack(t, lane_idx)
            piecewise_angle.appendTrack(t, lane_idx)
            x, y, a = t.getNext()

        # Centerline (orange) for reference
        track_length = piecewise_curvature.getLength()
        s_vals = np.linspace(0, track_length, 1000)[:-1]
        coords_list = [piecewise_position.get(s_i) for s_i in s_vals]
        coords_list = [m_to_px(self.canvas, x, y) for x, y in coords_list]

        if draw_parametric_curve:
            self.canvas.create_line(
                *coords_list,
                fill="darkorange",
                width=10,
                smooth=True
            )

        # Create car on the inner lane center
        self.cars.append(
            Car(
                initial_x,
                initial_y + lane_y,
                0.0,
                car1_img,
                "car 1",
                piecewise_curvature,
                piecewise_angle,
                piecewise_position,
                track_length,
            )
        )

        # Apply initial slider values (全是最小值，包括 voltage=0)
        if self.cars:
            car = self.cars[0]
            p = self.parameters
            car.iv = p.get("voltage", car.iv)
            car.mass = p.get("mass", 70.0) / 1000.0
            car.us = p.get("static_f", car.us)
            car.ud = p.get("dynamic_f", car.ud)
            car.wra = p.get("wheel_r", 4.0) / 1000.0
            car.kt = p.get("torque_c", car.kt)
            car.bemf = p.get("back_emf", car.bemf)
            car.gear_ratio = p.get("gear_ratio", car.gear_ratio)
            car.efficiency = p.get("efficiency", 80.0) / 100.0
            car.mag_param = p.get("max_energy", 0.0)

    # ----------------------------------------------------------
    #   Main redraw loop
    # ----------------------------------------------------------
    def redraw(self) -> None:
        for car in self.cars:
            car.tick(deltat)
            car.draw(self.canvas)

        self.parent.after(int(deltat * 1000), self.redraw)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
