import tkinter as tk
import time
from tkinter import ttk
from PIL import Image, ImageTk

from track import *
from config import *
from car import Car, car1_img

deltat = 0.05  # 50 ms

sw = 1600
sh = 800

piecewise_function_xy1 = None


class App:
    # (label, min, max, resolution, unit, internal_name)
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
        self.cars: list[Car] = []

        # Layout: left panel (controls), right panel (canvas)
        self.parent.grid_columnconfigure(0, weight=3)  # control panel
        self.parent.grid_columnconfigure(1, weight=7)  # canvas / simulation
        self.parent.grid_rowconfigure(0, weight=1)

        self.setup_control_panel()

        self.canvas = tk.Canvas(parent, bg="white")
        self.canvas.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # Build the track and car after Tk has finished layout
        self.parent.after(50, self.initCircuit)

        # Start redraw loop
        self.parent.after(int(deltat * 1000), self.redraw)

    # ------------------------------------------------------------------
    #  UI: left control panel
    # ------------------------------------------------------------------
    def setup_control_panel(self):
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
            font=("Arial", 16, "bold"),
        ).grid(row=0, column=0, columnspan=3, pady=(0, 15), sticky="w")

        self.sliders_container = ttk.Frame(self.control_frame)
        self.sliders_container.grid(row=1, column=0, columnspan=3, sticky="nsew")

        self.create_sliders(self.sliders_container)

    def create_sliders(self, parent: ttk.Frame):
        parent.grid_columnconfigure(0, weight=1)  # label
        parent.grid_columnconfigure(1, weight=3)  # slider
        parent.grid_columnconfigure(2, weight=1)  # value

        row_index = 0
        for label_text, min_val, max_val, resolution, unit, var_name in self.param_definitions:
            ttk.Label(parent, text=f"{label_text}:").grid(
                row=row_index, column=0, padx=5, pady=5, sticky="w"
            )

            # Choose a reasonable initial value for each parameter
            initial_value = min_val
            if var_name == "mass":
                initial_value = 70.0
            elif var_name == "static_f":
                initial_value = 0.3
            elif var_name == "dynamic_f":
                initial_value = 0.2
            elif var_name == "wheel_r":
                initial_value = 4.0
            elif var_name == "torque_c":
                initial_value = 0.8
            elif var_name == "back_emf":
                initial_value = 0.003
            elif var_name == "gear_ratio":
                initial_value = 2.5
            elif var_name == "efficiency":
                initial_value = 80.0

            self.parameters[var_name] = initial_value

            value_label = ttk.Label(
                parent,
                text=f"{initial_value:.4f} {unit}",
                width=12,
            )
            value_label.grid(row=row_index, column=2, padx=5, pady=5, sticky="e")

            def update_value_wrapper(name, u, label_widget):
                return lambda val: self.update_value(name, u, label_widget, val)

            slider = ttk.Scale(
                parent,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                command=update_value_wrapper(var_name, unit, value_label),
            )
            slider.set(initial_value)
            slider.grid(row=row_index, column=1, padx=5, pady=5, sticky="ew")

            row_index += 1

    def update_value(self, var_name, unit, value_label, value):
        """Callback for sliders: update internal dict and car parameters."""
        new_value = float(value)
        self.parameters[var_name] = new_value

        # Format the displayed value
        if var_name == "back_emf":
            formatted_value = f"{new_value:.4f}"
        elif new_value == round(new_value):
            formatted_value = f"{int(new_value)}"
        else:
            formatted_value = (
                f"{new_value:.1f}" if (new_value % 1) == 0.0 else f"{new_value:.2f}"
            )

        value_label.config(text=f"{formatted_value} {unit}")

        # If a car already exists, propagate parameter changes to it
        if self.cars:
            car = self.cars[0]
            if var_name == "voltage":
                car.iv = new_value
            elif var_name == "mass":
                car.mass = new_value / 1000.0  # g -> kg
            elif var_name == "static_f":
                car.us = new_value
            elif var_name == "dynamic_f":
                car.ud = new_value
            elif var_name == "wheel_r":
                car.wra = new_value / 1000.0  # mm -> m
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

    # ------------------------------------------------------------------
    #  Track + car initialization
    # ------------------------------------------------------------------
    def initCircuit(self):
        """Build the track and create the car."""
        global piecewise_function_xy1

        piecewise_function_t1 = CurvaturePiecewiseFunction()
        piecewise_function_xy1 = PositionPiecewiseFunction()
        piecewise_function_a = AnglePiecewiseFunction()

        lane_idx = 0
        initial_x, initial_y = -100 / 1000.0, -350 / 1000.0

        if lane_idx == 0:
            lane_y = (LANE_SPACING / 2.0 + LANE_SPACING) / 1000.0
        else:
            lane_y = (LANE_SPACING / 2.0) / 1000.0

        draw_tarmac = True
        draw_parametric_curve = True

        x, y, a = initial_x, initial_y, 0.0

        # Straight + curves, same layout as the original code
        t = C8205Track(x, y, a)
        if draw_tarmac:
            t.draw(self.canvas)
        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        for _ in range(4):
            t = C8204Track(x, y, a, "L")
            if draw_tarmac:
                t.draw(self.canvas)
            piecewise_function_t1.appendTrack(t, lane_idx)
            piecewise_function_xy1.appendTrack(t, lane_idx)
            piecewise_function_a.appendTrack(t, lane_idx)
            x, y, a = t.getNext()

        t = C8205Track(x, y, a)
        if draw_tarmac:
            t.draw(self.canvas)
        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        for _ in range(4):
            t = C8204Track(x, y, a, "L")
            if draw_tarmac:
                t.draw(self.canvas)
            piecewise_function_t1.appendTrack(t, lane_idx)
            piecewise_function_xy1.appendTrack(t, lane_idx)
            piecewise_function_a.appendTrack(t, lane_idx)
            x, y, a = t.getNext()

        # Parametric centerline (orange)
        s = np.linspace(0, piecewise_function_t1.getLength(), 1000)[0:-1]
        coords_list = [piecewise_function_xy1.get(s_i) for s_i in s]
        coords_list = [m_to_px(self.canvas, xx, yy) for xx, yy in coords_list]

        if draw_parametric_curve:
            self.canvas.create_line(
                *coords_list, fill="darkorange", width=10, smooth=True
            )

        # Create car on the selected lane
        car = Car(
            initial_x,
            initial_y + lane_y,
            0.0,
            car1_img,
            "car 1",
            piecewise_function_t1,
            piecewise_function_a,
            piecewise_function_xy1,
        )
        self.cars.append(car)

        # Apply initial slider values to the car
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

    # ------------------------------------------------------------------
    #  Main redraw / simulation loop
    # ------------------------------------------------------------------
    def redraw(self):
        """Main simulation loop driven by Tkinter's after()."""
        for car in self.cars:
            car.tick(deltat)
            car.draw(self.canvas)

        # Schedule next frame
        self.parent.after(int(deltat * 1000), self.redraw)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
