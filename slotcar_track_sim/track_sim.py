import tkinter as tk
import time
from tkinter import ttk
from PIL import Image, ImageTk

from track import *
from config import *
from car import *

deltat = 0.05  # 50 ms

sw = 1600
sh = 800

piecewise_function_xy1 = None


class App:

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

    def __init__(self, parent):
        self.parent = parent

        self.parent.title("Simulation")
        self.parent.geometry(f"{sw}x{sh}")

        self.parameters = {}
        self.cars = []

        # layout: left panel (controls), right panel (canvas)
        self.parent.grid_columnconfigure(0, weight=3)  # Control Panel
        self.parent.grid_columnconfigure(1, weight=7)  # Canvas/Simulation Area
        self.parent.grid_rowconfigure(0, weight=1)

        self.setup_control_panel()

        self.canvas = tk.Canvas(parent, bg='white')
        self.canvas.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # 等待窗口布局完成后再画赛道，否则 canvas 尺寸是 1x1 会导致跑道位置错乱
        self.parent.after(50, self.initCircuit)

        # 启动重绘循环
        self.parent.after(int(deltat * 1000), self.redraw)

    def setup_control_panel(self):
        """Sets up the left-side panel for parameter control."""
        self.control_frame = ttk.Frame(self.parent,
                                       padding="10 10 10 10",
                                       relief=tk.RAISED)
        self.control_frame.grid(row=0, column=0, sticky="nsew")
        self.control_frame.grid_rowconfigure(0, weight=0)  # Title row
        self.control_frame.grid_rowconfigure(1, weight=1)  # Sliders row

        ttk.Label(self.control_frame, text="System Parameters",
                  font=("Arial", 16, "bold")).grid(row=0, column=0,
                                                    columnspan=3,
                                                    pady=(0, 15),
                                                    sticky="w")

        self.sliders_container = ttk.Frame(self.control_frame)
        self.sliders_container.grid(row=1, column=0, columnspan=3,
                                    sticky="nsew")

        self.create_sliders(self.sliders_container)

    def create_sliders(self, parent):
        """Creates and places all parameter sliders and labels."""
        parent.grid_columnconfigure(0, weight=1)  # Label
        parent.grid_columnconfigure(1, weight=3)  # Slider
        parent.grid_columnconfigure(2, weight=1)  # Value

        row_index = 0
        for label_text, min_val, max_val, resolution, unit, var_name in self.param_definitions:
            ttk.Label(parent, text=f"{label_text}:").grid(
                row=row_index, column=0, padx=5, pady=5, sticky="w"
            )

            initial_value = min_val
            self.parameters[var_name] = initial_value
            value_label = ttk.Label(parent,
                                    text=f"{initial_value:.4f} {unit}",
                                    width=12)
            value_label.grid(row=row_index, column=2, padx=5, pady=5,
                             sticky="e")

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

    def update_value(self, var_name, unit, value_label, value):
        """Slider callback: update internal parameters and car object."""
        new_value = float(value)
        self.parameters[var_name] = new_value

        # Format display
        if var_name == "back_emf":
            formatted_value = f"{new_value:.4f}"
        elif new_value == round(new_value):
            formatted_value = f"{int(new_value)}"
        else:
            formatted_value = (f"{new_value:.1f}"
                               if (new_value % 1) == 0.0
                               else f"{new_value:.2f}")

        value_label.config(text=f"{formatted_value} {unit}")

        # Push to car if it already exists
        if self.cars:
            car = self.cars[0]
            if var_name == 'voltage':
                car.iv = new_value
            elif var_name == 'mass':
                car.mass = new_value / 1000.0       # g -> kg
            elif var_name == 'static_f':
                car.us = new_value
            elif var_name == 'dynamic_f':
                car.ud = new_value
            elif var_name == 'wheel_r':
                car.wra = new_value / 1000.0        # mm -> m
            elif var_name == 'torque_c':
                car.kt = new_value
            elif var_name == 'back_emf':
                car.bemf = new_value
            elif var_name == 'gear_ratio':
                car.gear_ratio = new_value
            elif var_name == 'efficiency':
                car.efficiency = new_value / 100.0  # % -> [0..1]

    def initCircuit(self):
        """Builds the track and creates the car."""
        global piecewise_function_xy1

        piecewise_function_t1 = CurvaturePiecewiseFunction()
        piecewise_function_xy1 = PositionPiecewiseFunction()
        piecewise_function_a = AnglePiecewiseFunction()

        lane_idx = 0
        initial_x, initial_y = -100 / 1000, -350 / 1000

        if lane_idx == 0:
            lane_y = (LANE_SPACING / 2 + LANE_SPACING) / 1000
        elif lane_idx == 1:
            lane_y = (LANE_SPACING / 2) / 1000

        drawTarmac = True
        drawParametricCurve = True

        x, y, a = initial_x, initial_y, 0

        # Straight + curves as in original code
        t = C8205Track(x, y, a)
        if drawTarmac:
            t.draw(self.canvas)
        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        for _ in range(4):
            t = C8204Track(x, y, a, 'L')
            if drawTarmac:
                t.draw(self.canvas)
            piecewise_function_t1.appendTrack(t, lane_idx)
            piecewise_function_xy1.appendTrack(t, lane_idx)
            piecewise_function_a.appendTrack(t, lane_idx)
            x, y, a = t.getNext()

        t = C8205Track(x, y, a)
        if drawTarmac:
            t.draw(self.canvas)
        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        for _ in range(4):
            t = C8204Track(x, y, a, 'L')
            if drawTarmac:
                t.draw(self.canvas)
            piecewise_function_t1.appendTrack(t, lane_idx)
            piecewise_function_xy1.appendTrack(t, lane_idx)
            piecewise_function_a.appendTrack(t, lane_idx)
            x, y, a = t.getNext()

        # parametric center line (orange)
        s = np.linspace(0, piecewise_function_t1.getLength(), 1000)[0:-1]
        coords_list = [piecewise_function_xy1.get(s_i) for s_i in s]
        coords_list = [m_to_px(self.canvas, x, y) for x, y in coords_list]

        if drawParametricCurve:
            self.canvas.create_line(*coords_list,
                                    fill="darkorange",
                                    width=10,
                                    smooth=True)

        # Create car
        self.cars.append(
            Car(initial_x, (initial_y + lane_y), 0,
                car1_img, 'car 1',
                piecewise_function_t1,
                piecewise_function_a,
                piecewise_function_xy1)
        )

        # Apply initial slider values to the car
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

    def redraw(self):
        """Main simulation loop driven by Tkinter's after()."""
        for car in self.cars:
            car.tick(deltat)
            car.draw(self.canvas)

        # schedule next frame
        self.parent.after(int(deltat * 1000), self.redraw)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
