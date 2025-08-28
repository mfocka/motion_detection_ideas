import csv
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox
import time
import collections
import queue
import traceback
from typing import Optional, Callable
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

from console_reader import MotionConsole

class LivePlotter:
    def __init__(self, max_points=500):
        self.max_points = max_points
        self.start_time = time.time()
        self.clear_data()

    def add_data(self, parsed_data):
        try:
            current_time = time.time() - self.start_time
            if not parsed_data or "type" not in parsed_data:
                return

            data_type = parsed_data["type"]
            if data_type == "ANGLES":
                self.time_angle.append(current_time)
                self.altitude.append(parsed_data["altitude"])
                self.azimuth.append(parsed_data["azimuth"])
                self.state.append(parsed_data["state"])
            elif data_type == "RAW_DATA":
                self.time_sensor.append(current_time)
                self.accel_x.append(parsed_data["accel_x"])
                self.accel_y.append(parsed_data["accel_y"])
                self.accel_z.append(parsed_data["accel_z"])
                self.gyro_x.append(parsed_data["gyro_x"])
                self.gyro_y.append(parsed_data["gyro_y"])
                self.gyro_z.append(parsed_data["gyro_z"])
            elif data_type == "GYRO_BIAS_MDI":
                self.time_bias.append(current_time)
                self.mdi_bias_x.append(parsed_data["bias_x"])
                self.mdi_bias_y.append(parsed_data["bias_y"])
                self.mdi_bias_z.append(parsed_data["bias_z"])
            elif data_type == "GYRO_BIAS_MFX":
                # Only append time if we don't have recent data
                if not self.time_bias or current_time - self.time_bias[-1] > 0.5:
                    self.time_bias.append(current_time)
                # Ensure MFX arrays have same length as time_bias
                while len(self.mfx_bias_x) < len(self.time_bias):
                    self.mfx_bias_x.append(0.0)
                    self.mfx_bias_y.append(0.0)
                    self.mfx_bias_z.append(0.0)
                # Update the last values
                if self.mfx_bias_x:
                    self.mfx_bias_x[-1] = parsed_data["bias_x"]
                    self.mfx_bias_y[-1] = parsed_data["bias_y"]
                    self.mfx_bias_z[-1] = parsed_data["bias_z"]
        except Exception as e:
            print(f"Error adding data: {e}")

    def get_data_snapshot(self):
        try:
            # Ensure all bias arrays have the same length
            max_bias_len = max(len(self.time_bias), len(self.mdi_bias_x), len(self.mfx_bias_x))
            
            # Pad shorter arrays with zeros
            while len(self.mdi_bias_x) < max_bias_len:
                self.mdi_bias_x.append(0.0)
                self.mdi_bias_y.append(0.0)
                self.mdi_bias_z.append(0.0)
            
            while len(self.mfx_bias_x) < max_bias_len:
                self.mfx_bias_x.append(0.0)
                self.mfx_bias_y.append(0.0)
                self.mfx_bias_z.append(0.0)
            
            return {
                "time_angle": list(self.time_angle),
                "altitude": list(self.altitude),
                "azimuth": list(self.azimuth),
                "state": self.state[-1] if self.state else "N/A",
                "time_sensor": list(self.time_sensor),
                "accel_x": list(self.accel_x),
                "accel_y": list(self.accel_y),
                "accel_z": list(self.accel_z),
                "gyro_x": list(self.gyro_x),
                "gyro_y": list(self.gyro_y),
                "gyro_z": list(self.gyro_z),
                "time_bias": list(self.time_bias),
                "mdi_bias_x": list(self.mdi_bias_x),
                "mdi_bias_y": list(self.mdi_bias_y),
                "mdi_bias_z": list(self.mdi_bias_z),
                "mfx_bias_x": list(self.mfx_bias_x),
                "mfx_bias_y": list(self.mfx_bias_y),
                "mfx_bias_z": list(self.mfx_bias_z),
            }
        except Exception as e:
            print(f"Error getting data snapshot: {e}")
            return self._get_empty_snapshot()

    def _get_empty_snapshot(self):
        return {
            "time_angle": [], "altitude": [], "azimuth": [], "state": "N/A",
            "time_sensor": [], "accel_x": [], "accel_y": [], "accel_z": [],
            "gyro_x": [], "gyro_y": [], "gyro_z": [],
            "time_bias": [], "mdi_bias_x": [], "mdi_bias_y": [], "mdi_bias_z": [],
            "mfx_bias_x": [], "mfx_bias_y": [], "mfx_bias_z": []
        }

    def clear_data(self):
        self.start_time = time.time()
        self.time_angle = collections.deque(maxlen=self.max_points)
        self.altitude = collections.deque(maxlen=self.max_points)
        self.azimuth = collections.deque(maxlen=self.max_points)
        self.state = collections.deque(maxlen=1)
        self.time_sensor = collections.deque(maxlen=self.max_points)
        self.accel_x = collections.deque(maxlen=self.max_points)
        self.accel_y = collections.deque(maxlen=self.max_points)
        self.accel_z = collections.deque(maxlen=self.max_points)
        self.gyro_x = collections.deque(maxlen=self.max_points)
        self.gyro_y = collections.deque(maxlen=self.max_points)
        self.gyro_z = collections.deque(maxlen=self.max_points)
        self.time_bias = collections.deque(maxlen=self.max_points)
        self.mdi_bias_x = collections.deque(maxlen=self.max_points)
        self.mdi_bias_y = collections.deque(maxlen=self.max_points)
        self.mdi_bias_z = collections.deque(maxlen=self.max_points)
        self.mfx_bias_x = collections.deque(maxlen=self.max_points)
        self.mfx_bias_y = collections.deque(maxlen=self.max_points)
        self.mfx_bias_z = collections.deque(maxlen=self.max_points)

class LiveConsoleApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Motion Sensor Live Plotter")
        self.root.geometry("1600x1000")

        self.plotter = LivePlotter()
        self.console_output = collections.deque(maxlen=208)
        self.console = MotionConsole(rx_callback=self._process_line)
        self.connected = False
        self.paused = False
        
        # Animation control
        self.animation_running = False

        # Logging
        self.logging_enabled = False
        self.log_file = None
        self.log_writer = None
        self.log_start_time = None

        self.setup_ui()
        self.setup_plots()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def toggle_logging(self):
        if not self.logging_enabled:
            # Start logging
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_filename = f"motion_log_{timestamp}.csv"
            
            try:
                self.log_file = open(log_filename, 'w', newline='')
                fieldnames = ['timestamp', 'type', 'data']
                self.log_writer = csv.DictWriter(self.log_file, fieldnames=fieldnames)
                self.log_writer.writeheader()
                self.log_start_time = time.time()
                self.logging_enabled = True
                self.log_button.config(text="Stop Logging")
                self.status_label.config(text=f"Logging to {log_filename}")
                print(f"Started logging to {log_filename}")
            except Exception as e:
                messagebox.showerror("Logging Error", f"Failed to start logging: {e}")
        else:
            # Stop logging
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.log_writer = None
            self.logging_enabled = False
            self.log_button.config(text="Start Logging")
            print("Stopped logging")

    def _log_data(self, line):
        if self.logging_enabled and self.log_writer:
            try:
                timestamp = time.time() - self.log_start_time
                self.log_writer.writerow({
                    'timestamp': timestamp,
                    'type': 'raw',
                    'data': line.strip()
                })
                self.log_file.flush()
            except Exception as e:
                print(f"Error logging data: {e}")

    def setup_ui(self):
        top_frame = ttk.Frame(self.root)
        top_frame.pack(side="top", fill="x", padx=10, pady=5)
        
        conn_frame = ttk.LabelFrame(top_frame, text="Serial Connection")
        conn_frame.pack(side="left", fill="y", padx=(0, 10))
        ttk.Label(conn_frame, text="Port:").pack(side="left", padx=5, pady=5)
        self.port_combobox = ttk.Combobox(conn_frame, values=self.console.get_available_ports(), width=15)
        self.port_combobox.pack(side="left", padx=5, pady=5)
        if self.console.get_available_ports():
            self.port_combobox.set(self.console.get_available_ports()[0])
        ttk.Label(conn_frame, text="Baud:").pack(side="left", padx=5, pady=5)
        self.baud_combobox = ttk.Combobox(conn_frame, values=[9600, 57600, 115200, 921600], width=10)
        self.baud_combobox.set("115200")
        self.baud_combobox.pack(side="left", padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=5, pady=5)
        
        control_frame = ttk.LabelFrame(top_frame, text="Controls & Status")
        control_frame.pack(side="left", fill="y", padx=(0, 10))
        self.pause_button = ttk.Button(control_frame, text="Pause", command=self.toggle_pause, state="disabled")
        self.pause_button.pack(side="left", padx=5, pady=5)
        self.clear_button = ttk.Button(control_frame, text="Clear Plots", command=self.clear_plots)
        self.clear_button.pack(side="left", padx=5, pady=5)
        self.calibrate_button = ttk.Button(control_frame, text="Calibrate", command=self.send_calibrate_command, state="disabled")
        self.calibrate_button.pack(side="left", padx=5, pady=5)
        # Add logging button
        self.log_button = ttk.Button(control_frame, text="Start Logging", command=self.toggle_logging, state="disabled")
        self.log_button.pack(side="left", padx=5, pady=5)
        # Add restart animation button
        self.restart_button = ttk.Button(control_frame, text="Restart Animation", command=self.restart_animation)
        self.restart_button.pack(side="left", padx=5, pady=5)
        self.status_label = ttk.Label(control_frame, text="Status: Disconnected", font=("Segoe UI", 10, "bold"))
        self.status_label.pack(side="left", padx=10, pady=5)
        
        main_frame = ttk.Frame(self.root)
        main_frame.pack(side="top", fill="both", expand=True, padx=10, pady=5)
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=3)
        main_frame.grid_columnconfigure(1, weight=2)
        self.plot_frame = ttk.Frame(main_frame)
        self.plot_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        
        console_frame = ttk.LabelFrame(main_frame, text="Console Output")
        console_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        console_frame.grid_rowconfigure(0, weight=1)
        console_frame.grid_columnconfigure(0, weight=1)
        self.console_text = tk.Text(console_frame, wrap="word", state="disabled", height=10, bg="#2b2b2b", fg="white", font=("Consolas", 9))
        self.console_text.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        self.console_scrollbar = ttk.Scrollbar(console_frame, command=self.console_text.yview)
        self.console_scrollbar.grid(row=0, column=1, sticky="ns")
        self.console_text.config(yscrollcommand=self.console_scrollbar.set)
        
        command_entry_frame = ttk.Frame(console_frame)
        command_entry_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.command_entry = ttk.Entry(command_entry_frame)
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", self.send_command_from_entry)
        self.send_button = ttk.Button(command_entry_frame, text="Send", command=self.send_command_from_entry)
        self.send_button.pack(side="right")

    def setup_plots(self):
        self.fig = Figure(figsize=(14, 10), dpi=100)
        self.fig.tight_layout(pad=4.0)
        
        self.ax_angles = self.fig.add_subplot(3, 2, 1)
        self.ax_accel = self.fig.add_subplot(3, 2, 2)
        self.ax_gyro = self.fig.add_subplot(3, 2, 3)
        self.ax_bias = self.fig.add_subplot(3, 2, 4)
        self.ax_values = self.fig.add_subplot(3, 2, (5, 6))
        
        
        # Autoscale variables
        self.autoscale_angles = tk.BooleanVar(value=True)
        self.autoscale_accel = tk.BooleanVar(value=True)
        self.autoscale_gyro = tk.BooleanVar(value=True)
        self.autoscale_bias = tk.BooleanVar(value=True)

        # Setup angle plot
        self.ax_angles.set_title("Altitude & Azimuth")
        self.ax_angles.grid(True)
        self.ax_angles.set_xlabel("Time (s)")
        self.ax_angles.set_ylabel("Angle (degrees)")
        self.line_alt, = self.ax_angles.plot([], [], label="Altitude", color='dodgerblue', linewidth=2)
        self.line_az, = self.ax_angles.plot([], [], label="Azimuth", color='orangered', linewidth=2)
        self.ax_angles.legend(loc='upper left')
        
        # Setup accelerometer plot
        self.ax_accel.set_title("Accelerometer (mg)")
        self.ax_accel.grid(True)
        self.ax_accel.set_xlabel("Time (s)")
        self.ax_accel.set_ylabel("Acceleration (mg)")
        self.line_ax, = self.ax_accel.plot([], [], label="X", linewidth=1.5)
        self.line_ay, = self.ax_accel.plot([], [], label="Y", linewidth=1.5)
        self.line_az_accel, = self.ax_accel.plot([], [], label="Z", linewidth=1.5)
        self.ax_accel.legend(loc='upper left')
        
        # Setup gyroscope plot
        self.ax_gyro.set_title("Gyroscope (dps)")
        self.ax_gyro.grid(True)
        self.ax_gyro.set_xlabel("Time (s)")
        self.ax_gyro.set_ylabel("Angular Rate (dps)")
        self.line_gx, = self.ax_gyro.plot([], [], label="X", linewidth=1.5)
        self.line_gy, = self.ax_gyro.plot([], [], label="Y", linewidth=1.5)
        self.line_gz, = self.ax_gyro.plot([], [], label="Z", linewidth=1.5)
        self.ax_gyro.legend(loc='upper left')
        
        # Setup bias plot
        self.ax_bias.set_title("Gyroscope Bias (dps)")
        self.ax_bias.grid(True)
        self.ax_bias.set_xlabel("Time (s)")
        self.ax_bias.set_ylabel("Bias (dps)")
        self.line_mdi_bias_x, = self.ax_bias.plot([], [], label="MDI X", linewidth=1.5, linestyle='-')
        self.line_mdi_bias_y, = self.ax_bias.plot([], [], label="MDI Y", linewidth=1.5, linestyle='-')
        self.line_mdi_bias_z, = self.ax_bias.plot([], [], label="MDI Z", linewidth=1.5, linestyle='-')
        self.line_mfx_bias_x, = self.ax_bias.plot([], [], label="MFX X", linewidth=1.5, linestyle='--')
        self.line_mfx_bias_y, = self.ax_bias.plot([], [], label="MFX Y", linewidth=1.5, linestyle='--')
        self.line_mfx_bias_z, = self.ax_bias.plot([], [], label="MFX Z", linewidth=1.5, linestyle='--')
        self.ax_bias.legend(loc='upper left', ncol=2, fontsize=8)
        
        # Setup values display
        self.ax_values.axis('off')
        self.values_text = self.ax_values.text(0.05, 0.95, '', fontsize=11, family='monospace', va='top')
        
        self.canvas = FigureCanvasTkAgg(self.fig, self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Start animation
        self.start_animation()
        
        # Checkboxes for autoscale
        checkbox_frame = ttk.Frame(self.plot_frame)
        checkbox_frame.pack(fill='x', pady=5)
        ttk.Checkbutton(checkbox_frame, text="Autoscale Angles", variable=self.autoscale_angles).pack(side='left', padx=20)
        ttk.Checkbutton(checkbox_frame, text="Autoscale Accel", variable=self.autoscale_accel).pack(side='left', padx=20)
        ttk.Checkbutton(checkbox_frame, text="Autoscale Gyro", variable=self.autoscale_gyro).pack(side='left', padx=20)
        ttk.Checkbutton(checkbox_frame, text="Autoscale Bias", variable=self.autoscale_bias).pack(side='left', padx=10)
    def start_animation(self):
        """Start or restart the animation"""
        try:
            if hasattr(self, 'ani') and self.ani is not None:
                self.ani.event_source.stop()
            
            self.ani = FuncAnimation(
                self.fig, 
                self.update_plots, 
                blit=False,  # Changed to False for better reliability
                cache_frame_data=False,
                repeat=True
            )
            self.animation_running = True
            print("Animation started successfully")
        except Exception as e:
            print(f"Error starting animation: {e}")
            self.animation_running = False

    def restart_animation(self):
        """Manually restart the animation"""
        print("Restarting animation...")
        self.start_animation()

    def update_plots(self, frame):
        """Update all plots with current data"""
        try:
            data = self.plotter.get_data_snapshot()
            time_window = 5  # seconds
            
            # Update angle plot
            t_angle, alt, az = data["time_angle"], data["altitude"], data["azimuth"]
            self.line_alt.set_data(t_angle, alt)
            self.line_az.set_data(t_angle, az)
            
            if t_angle:
                # Set time axis
                x_min = max(0, t_angle[-1] - time_window)
                x_max = t_angle[-1] + 1
                self.ax_angles.set_xlim(x_min, x_max)
                
                # Auto-scale Y axis for angles
                if self.autoscale_angles.get() and (alt or az):
                    all_angles = list(alt) + list(az)
                    if all_angles:
                        y_min = min(all_angles) - 5
                        y_max = max(all_angles) + 5
                        # Ensure minimum range
                        if y_max - y_min < 10:
                            center = (y_max + y_min) / 2
                            y_min = center - 5
                            y_max = center + 5
                        self.ax_angles.set_ylim(y_min, y_max)

            # Update sensor plots
            t_sensor = data["time_sensor"]
            ax, ay, az_acc = data["accel_x"], data["accel_y"], data["accel_z"]
            gx, gy, gz = data["gyro_x"], data["gyro_y"], data["gyro_z"]
            
            # Update accelerometer
            self.line_ax.set_data(t_sensor, ax)
            self.line_ay.set_data(t_sensor, ay)
            self.line_az_accel.set_data(t_sensor, az_acc)
            
            # Update gyroscope
            self.line_gx.set_data(t_sensor, gx)
            self.line_gy.set_data(t_sensor, gy)
            self.line_gz.set_data(t_sensor, gz)

            # Update bias plot - this is where the fix is most important
            t_bias = data["time_bias"]
            mdi_bx, mdi_by, mdi_bz = data["mdi_bias_x"], data["mdi_bias_y"], data["mdi_bias_z"]
            mfx_bx, mfx_by, mfx_bz = data["mfx_bias_x"], data["mfx_bias_y"], data["mfx_bias_z"]
            
            # Set data for all bias lines, even if MFX data is empty (will show as empty lines)
            self.line_mdi_bias_x.set_data(t_bias, mdi_bx)
            self.line_mdi_bias_y.set_data(t_bias, mdi_by)
            self.line_mdi_bias_z.set_data(t_bias, mdi_bz)
            self.line_mfx_bias_x.set_data(t_bias, mfx_bx)
            self.line_mfx_bias_y.set_data(t_bias, mfx_by)
            self.line_mfx_bias_z.set_data(t_bias, mfx_bz)
            
            if t_bias:
                # Set time axis
                x_min = max(0, t_bias[-1] - time_window)
                x_max = t_bias[-1] + 1
                self.ax_bias.set_xlim(x_min, x_max)
                
                # Auto-scale Y axis for bias (only use non-zero values for scaling)
                if self.autoscale_bias.get():
                    all_bias = [x for x in list(mdi_bx) + list(mdi_by) + list(mdi_bz) + 
                               list(mfx_bx) + list(mfx_by) + list(mfx_bz) if x != 0.0]
                    if all_bias:
                        y_min = min(all_bias) - 0.1
                        y_max = max(all_bias) + 0.1
                        # Ensure minimum range
                        if y_max - y_min < 0.2:
                            center = (y_max + y_min) / 2
                            y_min = center - 0.1
                            y_max = center + 0.1
                        self.ax_bias.set_ylim(y_min, y_max)   
            if t_sensor:
                # Set time axis for sensor plots
                x_min = max(0, t_sensor[-1] - time_window)
                x_max = t_sensor[-1] + 1
                self.ax_accel.set_xlim(x_min, x_max)
                self.ax_gyro.set_xlim(x_min, x_max)
                
                # Auto-scale Y axis for accelerometer
                if self.autoscale_accel.get():
                    all_accel = list(ax) + list(ay) + list(az_acc)
                    if all_accel:
                        y_min = min(all_accel) - 100
                        y_max = max(all_accel) + 100
                        # Ensure minimum range
                        if y_max - y_min < 200:
                            center = (y_max + y_min) / 2
                            y_min = center - 100
                            y_max = center + 100
                        self.ax_accel.set_ylim(y_min, y_max)
                
                # Auto-scale Y axis for gyroscope
                if self.autoscale_gyro.get():
                    all_gyro = list(gx) + list(gy) + list(gz)
                    if all_gyro:
                        y_min = min(all_gyro) - 10
                        y_max = max(all_gyro) + 10
                        # Ensure minimum range
                        if y_max - y_min < 20:
                            center = (y_max + y_min) / 2
                            y_min = center - 10
                            y_max = center + 10
                        self.ax_gyro.set_ylim(y_min, y_max)

            # Update values text
            txt = f"State: {data['state']}\n\n"
            txt += f"Altitude: {alt[-1]:>7.2f}°\n" if alt else "Altitude: N/A\n"
            txt += f"Azimuth:  {az[-1]:>7.2f}°\n\n" if az else "Azimuth:  N/A\n"
            txt += f"Accel X:  {ax[-1]:>7.1f} mg\n" if ax else "Accel X:  N/A\n"
            txt += f"Accel Y:  {ay[-1]:>7.1f} mg\n" if ay else "Accel Y:  N/A\n"
            txt += f"Accel Z:  {az_acc[-1]:>7.1f} mg\n\n" if az_acc else "Accel Z:  N/A\n"
            txt += f"Gyro X:   {gx[-1]:>7.1f} dps\n" if gx else "Gyro X:   N/A\n"
            txt += f"Gyro Y:   {gy[-1]:>7.1f} dps\n" if gy else "Gyro Y:   N/A\n"
            txt += f"Gyro Z:   {gz[-1]:>7.1f} dps\n" if gz else "Gyro Z:   N/A\n"
            if mdi_bx:
                txt += f"MDI Bias: X:{mdi_bx[-1]:>6.3f} Y:{mdi_by[-1]:>6.3f} Z:{mdi_bz[-1]:>6.3f}\n"
            else:
                txt += "MDI Bias: N/A\n"
            # Only show MFX bias if we have non-zero values
            if mfx_bx and any(x != 0.0 for x in mfx_bx):
                txt += f"MFX Bias: X:{mfx_bx[-1]:>6.3f} Y:{mfx_by[-1]:>6.3f} Z:{mfx_bz[-1]:>6.3f}"
            else:
                txt += "MFX Bias: N/A"
            self.values_text.set_text(txt)
            
            return []  # Return empty list for blit=False
            
        except Exception as e:
            print(f"Error updating plots: {e}")
            print(traceback.format_exc())
            # Try to restart animation on error
            if self.animation_running:
                self.root.after(1000, self.restart_animation)
            return []

    def _process_line(self, line):
        try:
            self._log_data(line)
            self.console_output.append(line)
            if not self.paused:
                parsed_data = self._parse_data_line(line)
                if parsed_data:
                    self.plotter.add_data(parsed_data)
            self.root.after_idle(self._update_console_text)
        except Exception as e:
            print(f"Error processing line: {e}")

    def _parse_data_line(self, line_full):
        try:
            # Handle lines with timestamp format [timestamp] data
            if "] " in line_full:
                line = line_full.split("] ", 1)[1]
            else:
                line = line_full
                
            parts = line.split(',')
            if not parts:
                return None
                
            prefix = parts[0].strip()
    
            if prefix == "ANGLES" and len(parts) >= 5:
                return {
                    "type": "ANGLES", 
                    "timestamp": int(parts[1]), 
                    "altitude": float(parts[2]), 
                    "azimuth": float(parts[3]), 
                    "state": parts[-1].strip()
                }
            elif prefix == "RAW_DATA" and len(parts) >= 8:
                return {
                    "type": "RAW_DATA", 
                    "timestamp": int(parts[1]), 
                    "accel_x": float(parts[2]), 
                    "accel_y": float(parts[3]), 
                    "accel_z": float(parts[4]), 
                    "gyro_x": float(parts[5]), 
                    "gyro_y": float(parts[6]), 
                    "gyro_z": float(parts[7])
                }
            elif prefix == "GYRO_BIAS_MDI" and len(parts) >= 5:
                return {
                    "type": "GYRO_BIAS_MDI",
                    "timestamp": int(parts[1]),
                    "bias_x": float(parts[2]),
                    "bias_y": float(parts[3]),
                    "bias_z": float(parts[4])
                }
            elif prefix == "GYRO_BIAS_MFX" and len(parts) >= 5:
                return {
                    "type": "GYRO_BIAS_MFX",
                    "timestamp": int(parts[1]),
                    "bias_x": float(parts[2]),
                    "bias_y": float(parts[3]),
                    "bias_z": float(parts[4])
                }
        except (ValueError, IndexError, AttributeError) as e:
            print(f"Error parsing line '{line_full}': {e}")
        return None

    def _update_console_text(self):
        try:
            self.console_text.config(state="normal")
            self.console_text.delete(1.0, tk.END)
            self.console_text.insert(tk.END, "\n".join(self.console_output))
            self.console_text.see(tk.END)
            self.console_text.config(state="disabled")
        except Exception as e:
            print(f"Error updating console text: {e}")

    def toggle_connection(self):
        try:
            if not self.console.is_connected():
                port = self.port_combobox.get()
                if not port:
                    messagebox.showerror("Connection Error", "No serial port selected.")
                    return
                baud = int(self.baud_combobox.get())
                if self.console.connect(port, baud):
                    self.connect_button.config(text="Disconnect")
                    self.status_label.config(text=f"Status: Connected to {port}", foreground="green")
                    self.port_combobox.config(state="disabled")
                    self.baud_combobox.config(state="disabled")
                    self.pause_button.config(state="normal")
                    self.calibrate_button.config(state="normal")
                    self.log_button.config(state="normal")
                    self.connected = True
                else:
                    messagebox.showerror("Connection Error", f"Failed to connect to {port}")
            else:
                self.console.disconnect()
                self.connect_button.config(text="Connect")
                self.status_label.config(text="Status: Disconnected", foreground="red")
                self.port_combobox.config(state="readonly")
                self.baud_combobox.config(state="readonly")
                self.pause_button.config(state="disabled", text="Pause")
                self.calibrate_button.config(state="disabled")
                self.log_button.config(state="disabled")
                self.connected = False
                self.paused = False
        except Exception as e:
            print(f"Error toggling connection: {e}")
            messagebox.showerror("Connection Error", f"Error: {e}")

    def toggle_pause(self):
        self.paused = not self.paused
        if self.paused:
            self.pause_button.config(text="Resume")
            self.status_label.config(text="Status: Paused", foreground="orange")
        else:
            self.pause_button.config(text="Pause")
            if self.console.is_connected():
                self.status_label.config(text=f"Status: Connected to {self.port_combobox.get()}", foreground="green")

    def clear_plots(self):
        try:
            self.plotter.clear_data()
            # Reset to default ranges
            self.ax_angles.set_ylim(-90, 90)
            self.ax_accel.set_ylim(-2000, 2000)
            self.ax_gyro.set_ylim(-250, 250)
            self.canvas.draw_idle()
            print("Plots cleared.")
        except Exception as e:
            print(f"Error clearing plots: {e}")

    def send_command_from_entry(self, event=None):
        try:
            command = self.command_entry.get()
            if command and self.console.is_connected():
                self.console.send_command(command)
                self.command_entry.delete(0, tk.END)
        except Exception as e:
            print(f"Error sending command: {e}")
            
    def send_calibrate_command(self):
        try:
            if self.console.is_connected():
                self.console.calibrate(alt_threshold=5.0, az_threshold=10.0)
        except Exception as e:
            print(f"Error sending calibrate command: {e}")

    def on_closing(self):
        try:
            print("Shutting down...")
            if hasattr(self, 'console') and self.console:
                self.console.disconnect()
            if hasattr(self, 'ani') and self.ani:
                self.ani.event_source.stop()
            self.root.quit()
            self.root.destroy()
        except Exception as e:
            print(f"Error during shutdown: {e}")

    def run(self):
        try:
            self.root.mainloop()
        except Exception as e:
            print(f"Error running application: {e}")
            traceback.print_exc()

if __name__ == "__main__":
    try:
        app = LiveConsoleApp()
        app.run()
    except Exception as e:
        print(f"Fatal error: {e}")
        traceback.print_exc()