import tkinter as tk
from tkinter import ttk
from tkinter import messagebox, filedialog
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
import csv
import math
import serial
# Sử dụng TkAgg làm backend cho matplotlib
matplotlib.use("TkAgg")

class TrajectoryApp:
    def __init__(self, root):
        self.ser=serial.Serial('/dev/ttyUSB0',57600)
        self.root = root
        self.root.title("Trajectory Tracker")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        # Khung nhập liệu cho x và y
        input_frame = ttk.Frame(root, padding="10")
        input_frame.grid(row=0, column=0, sticky="W")
        self.clear_button = ttk.Button(input_frame, text="Clear", command=self.clear_plot)
        self.clear_button.grid(row=0, column=7, padx=10, pady=5)
        # Nhãn và ô nhập cho x
        ttk.Label(input_frame, text="X:").grid(row=0, column=0, padx=5, pady=5, sticky="E")
        self.x_entry = ttk.Entry(input_frame, width=10)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)

        # Nhãn và ô nhập cho y
        ttk.Label(input_frame, text="Y:").grid(row=0, column=2, padx=5, pady=5, sticky="E")
        self.y_entry = ttk.Entry(input_frame, width=10)
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)

        # Nút Start
        self.start_button = ttk.Button(input_frame, text="Start", command=self.start_tracking)
        self.start_button.grid(row=0, column=4, padx=10, pady=5)

        # Nút Load Trajectory
        self.load_button = ttk.Button(input_frame, text="Load Trajectory", command=self.load_trajectory)
        self.load_button.grid(row=0, column=5, padx=10, pady=5)
        angle_frame = ttk.Frame(root, padding="10")
        angle_frame.grid(row=2, column=0, sticky="W")

        # Nhãn và hiển thị góc của servo 1
        ttk.Label(angle_frame, text="Servo 1 Angle:").grid(row=0, column=0, padx=5, pady=5, sticky="E")
        self.servo1_angle = ttk.Label(angle_frame, text="0.0")
        self.servo1_angle.grid(row=0, column=1, padx=5, pady=5)

        # Nhãn và hiển thị góc của servo 2
        ttk.Label(angle_frame, text="Servo 2 Angle:").grid(row=1, column=0, padx=5, pady=5, sticky="E")
        self.servo2_angle = ttk.Label(angle_frame, text="0.0")
        self.servo2_angle.grid(row=1, column=1, padx=5, pady=5)

        # Nhãn và hiển thị góc của servo 3
        ttk.Label(angle_frame, text="Servo 3 Angle:").grid(row=2, column=0, padx=5, pady=5, sticky="E")
        self.servo3_angle = ttk.Label(angle_frame, text="0.0")
        self.servo3_angle.grid(row=2, column=1, padx=5, pady=5)
        # Khung biểu đồ
        self.figure = plt.Figure(figsize=(12,6))
        
        # Biểu đồ quỹ đạo động
        self.ax_dynamic = self.figure.add_subplot(121)
        self.ax_dynamic.set_title("Dynamic Trajectory")
        self.ax_dynamic.set_xlabel("X")
        self.ax_dynamic.set_ylabel("Y")
        self.ax_dynamic.grid(True)
        self.line_dynamic, = self.ax_dynamic.plot([], [], 'b-', marker='o', markersize=5, color='blue')
        self.all_trajectory_x = []
        self.all_trajectory_y = []
        # Biểu đồ quỹ đạo từ file
        self.ax_loaded = self.figure.add_subplot(122)
        self.ax_loaded.set_title("Loaded Trajectory")
        self.ax_loaded.set_xlabel("X")
        self.ax_loaded.set_ylabel("Y")
        self.ax_loaded.grid(True)
        self.line_loaded, = self.ax_loaded.plot([], [], 'r-', marker='x', markersize=5, color='red')

        self.canvas = FigureCanvasTkAgg(self.figure, master=root)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=1, column=0, padx=10, pady=10)

        # Khởi tạo biến theo dõi
        self.tracking = False
        self.thread = None
        self.lock = threading.Lock()
        self.x = 0
        self.y = 0
        self.trajectory_x = []
        self.trajectory_y = []
        self.loaded_trajectory_x = []
        self.loaded_trajectory_y = []
    def send_serial(self,x,y,theta):
        str_send=str(x)+"/"+str(y)+"#"+str(theta)+";"
        self.ser.write(str_send.encode())
    def start_tracking(self):
        # Lấy giá trị x và y từ ô nhập liệu
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
        except ValueError:
            messagebox.showerror("Input Error", "Vui lòng nhập giá trị số hợp lệ cho X và Y.")
            return

        servo_angles = self.calculate_inverse_kinematics(x, y)
        self.servo1_angle.config(text=f"{servo_angles[0]:.2f}")
        self.servo2_angle.config(text=f"{servo_angles[1]:.2f}")
        self.servo3_angle.config(text=f"{servo_angles[2]:.2f}")

        with self.lock:
            self.target_x = x
            self.target_y = y
            self.send_serial(x,y,0)
            self.trajectory_x = []
            self.trajectory_y = []
            
        self.ax_dynamic.cla()
        self.ax_dynamic.grid(True)

        self.start_button.config(state='disabled')
        self.load_button.config(state='disabled')
        self.stop_button = ttk.Button(self.root, text="Stop", command=self.stop_tracking)
        self.stop_button.grid(row=0, column=6, padx=10, pady=5)

        self.tracking = True
        self.thread = threading.Thread(target=self.update_position)
        self.thread.start()
    def calculate_inverse_kinematics(self, x, y):
        # Giả sử chúng ta có mô hình đơn giản để tính góc servo dựa trên tọa độ x, y
        # Thay thế công thức này bằng mô hình động học ngược của 3RRR của bạn
        servo1_angle = math.atan2(y, x)
        servo2_angle = math.atan2(y, x + 1)
        servo3_angle = math.atan2(y, x - 1)
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode().strip()  # Đọc dữ liệu từ Serial và loại bỏ ký tự xuống dòng
            print(data)
        return servo1_angle * 180 / math.pi, servo2_angle * 180 / math.pi, servo3_angle * 180 / math.pi
    def stop_tracking(self):
        self.tracking = False
        self.start_button.config(state='normal')
        self.load_button.config(state='normal')
        self.stop_button.destroy()

    def update_position(self):
        while self.tracking:
           
            with self.lock:
                # Sử dụng thuật toán Bresenham để xấp xỉ đường thẳng từ vị trí hiện tại đến vị trí nhập vào
                points = self.bresenham(self.x, self.y, self.target_x, self.target_y)
            for point in points:
                if not self.tracking:
                    break
                time.sleep(0.1)
                with self.lock:
                    self.x, self.y = point
                    self.trajectory_x.append(self.x)
                    self.trajectory_y.append(self.y)
                    self.send_serial(self.x,self.y,0)
                servo_angles = self.calculate_inverse_kinematics(self.x, self.y)
                self.servo1_angle.config(text=f"{servo_angles[0]:.2f}")
                self.servo2_angle.config(text=f"{servo_angles[1]:.2f}")
                self.servo3_angle.config(text=f"{servo_angles[2]:.2f}")
                # trajectory_x = self.trajectory_x.copy()
                # trajectory_y = self.trajectory_y.copy()
                self.root.after(0, self.plot_dynamic_trajectory,self.trajectory_x,self.trajectory_x)
      
            # Cập nhật biểu đồ trên giao diện Tkinter phải được thực hiện trong luồng chính
    def bresenham(self, x1, y1, x2, y2):
        """Thuật toán Bresenham để xấp xỉ đường thẳng"""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return points
    def clear_plot(self):
        # Xóa toàn bộ quỹ đạo đã lưu và làm mới biểu đồ
        with self.lock:
            self.trajectory_x = []
            self.trajectory_y = []
        self.ax_dynamic.cla()
        self.ax_dynamic.set_title("Dynamic Trajectory")
        self.ax_dynamic.set_xlabel("X")
        self.ax_dynamic.set_ylabel("Y")
        self.ax_dynamic.grid(True)
        self.canvas.draw()
    def plot_dynamic_trajectory(self, x_data, y_data):
        """Vẽ lại quỹ đạo động, không xóa quỹ đạo cũ."""
        # self.ax_dynamic.cla()
        self.ax_dynamic.set_title("Dynamic Trajectory")
        self.ax_dynamic.set_xlabel("X")
        self.ax_dynamic.set_ylabel("Y")
        self.ax_dynamic.grid(True)
        self.ax_dynamic.plot(x_data, y_data, 'b-', marker='o', markersize=5, color='blue')
        self.canvas.draw()

    def load_trajectory(self):
        file_path = filedialog.askopenfilename(
            title="Chọn file quỹ đạo",
            filetypes=(("CSV files", "*.csv"), ("G-code files", "*.gcode *.nc *.tap"), ("All files", "*.*"))
        )
        if not file_path:
            return  # Người dùng đã hủy bỏ việc chọn file

        try:
            if file_path.lower().endswith('.csv'):
                loaded_x, loaded_y = self.parse_csv(file_path)
            elif file_path.lower().endswith(('.gcode', '.nc', '.tap')):
                loaded_x, loaded_y = self.parse_gcode(file_path)
            else:
                messagebox.showerror("File Error", "Định dạng file không được hỗ trợ.")
                return
        except Exception as e:
            messagebox.showerror("File Error", f"Không thể đọc file:\n{e}")
            return

        with self.lock:
            self.loaded_trajectory_x = loaded_x
            self.loaded_trajectory_y = loaded_y

        # Vẽ quỹ đạo đã tải trên biểu đồ thứ hai
        self.ax_loaded.cla()
        self.ax_loaded.set_title("Loaded Trajectory")
        self.ax_loaded.set_xlabel("X")
        self.ax_loaded.set_ylabel("Y")
        self.ax_loaded.grid(True)
        self.ax_loaded.plot(self.loaded_trajectory_x, self.loaded_trajectory_y, 'r-', marker='x', markersize=5, color='red')
        self.canvas.draw()
    def on_closing(self):
        """Dừng thread khi đóng chương trình"""
        self.tracking = False
        if self.thread is not None:
            self.thread.join()  # Đợi cho thread dừng hoàn toàn
        self.root.destroy()

    def parse_csv(self, file_path):
        x = []
        y = []
        with open(file_path, 'r', newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            if 'X' not in reader.fieldnames or 'Y' not in reader.fieldnames:
                raise ValueError("File CSV phải có các cột 'X' và 'Y'.")
            for row in reader:
                try:
                    x_val = float(row['X'])
                    y_val = float(row['Y'])
                    x.append(x_val)
                    y.append(y_val)
                except ValueError:
                    # Bỏ qua các dòng có giá trị không hợp lệ
                    continue
        return x, y
    def parse_gcode(self, file_path):
        x = []
        y = []
        with open(file_path, 'r') as f:
            for line in f:
                if line.startswith('G1') or line.startswith('G0'):
                    parts = line.strip().split()
                    x_val = None
                    y_val = None
                    for part in parts:
                        if part.startswith('X'):
                            try:
                                x_val = float(part[1:])
                            except ValueError:
                                pass
                        elif part.startswith('Y'):
                            try:
                                y_val = float(part[1:])
                            except ValueError:
                                pass
                    if x_val is not None and y_val is not None:
                        x.append(x_val)
                        y.append(y_val)
        return x, y


if __name__ == "__main__":
    root = tk.Tk()
    app = TrajectoryApp(root)
    root.mainloop()
    
