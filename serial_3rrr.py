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
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.ops import unary_union
import numpy as np
# Sử dụng TkAgg làm backend cho matplotlib
rb = 185  # ví dụ giá trị radius của base (rb)
rp = [45, 45, 45]  # ví dụ giá trị radius của platform (rp)
L1 = 142  # Chiều dài đoạn L1
L2 = 120  # Chiều dài đoạn L2
b1x =160
b1y = -98
b2x = 0
b2y = rb
b3x = -170
b3y = -98
home_offset_angle = [0, 0, 0]  # ví dụ giá trị
init=[36.54,27,40.126]
limit_angle_beta=math.radians(55.0)
origin_servo=[98.175990932417,99.63736216279389,101.76306703396516]
angle_sing_init=[65.34652095078408,63.95451633183546,69.9299546791449]
target = [0, 0, 0]  # ví dụ giá trị mục tiêu
def deg_to_rad(number):
    return number*math.pi/180.0
def rad_to_deg(number):
    return (number * 180.0)/(math.pi)
for i in range(0,3):
    home_offset_angle[i]=deg_to_rad(origin_servo[i]-init[i])
    target[i]=init[i]
# Hàm unwrap
def unwrap(theta):
    if theta < 0:
        theta = theta + 2 * math.pi
    return theta

# Hàm inverse kinematics cho robot 3RRR
def InvKinRRR(px, py, theta):
    a_angles = [-math.pi / 6, math.pi / 2, 7 * math.pi / 6]

    # Tính toán tọa độ các điểm a1, a2, a3
    a1x = px + rp[0] * math.cos(theta + a_angles[0])
    a1y = py + rp[0] * math.sin(theta + a_angles[0])
    a2x = px + rp[1] * math.cos(theta + a_angles[1])
    a2y = py + rp[1] * math.sin(theta + a_angles[1])
    a3x = px + rp[2] * math.cos(theta + a_angles[2])
    a3y = py + rp[2] * math.sin(theta + a_angles[2])

    # Tính khoảng cách a1b1, a2b2, a3b3
    a1b1 = math.sqrt((a1x - b1x) ** 2 + (a1y - b1y) ** 2)
    a2b2 = math.sqrt((a2x - b2x) ** 2 + (a2y - b2y) ** 2)
    a3b3 = math.sqrt((a3x - b3x) ** 2 + (a3y - b3y) ** 2)

    # Tính các góc alpha1, alpha2, alpha3
    alpha1 = math.acos((a1b1 ** 2 + L1 ** 2 - L2 ** 2) / (2 * L1 * a1b1))
    alpha2 = math.acos((a2b2 ** 2 + L1 ** 2 - L2 ** 2) / (2 * L1 * a2b2))
    alpha3 = math.acos((a3b3 ** 2 + L1 ** 2 - L2 ** 2) / (2 * L1 * a3b3))
    beta_sing_1=math.acos((L1**2+L2**2-a1b1**2)/(2*L1*L2))
    beta_sing_2 = math.acos((L1 ** 2 + L2 ** 2 - a2b2 ** 2) / (2 * L1 * L2))
    beta_sing_3 = math.acos((L1 ** 2 + L2 ** 2 - a3b3 ** 2) / (2 * L1 * L2))
    # print(rad_to_deg(beta_sing_2))
    if(beta_sing_1<limit_angle_beta or beta_sing_2<limit_angle_beta or beta_sing_3<limit_angle_beta):
        print("singularity in x:"+str(px) +" with y:"+str(py))
    # print("beta:")
    # print(rad_to_deg(beta_sing_1))
    # print(rad_to_deg(beta_sing_2))
    # print(rad_to_deg(beta_sing_3))
    # print("---")
    # Tính các góc psi1, psi2, psi3
    psi1 = math.atan2(a1y - b1y, a1x - b1x)
    psi2 = math.atan2(a2y - b2y, a2x - b2x)
    psi3 = math.atan2(a3y - b3y, a3x - b3x)

    # Unwrap các góc psi
    psi1 = unwrap(psi1)
    psi2 = unwrap(psi2)
    psi3 = unwrap(psi3)

    # Tính các góc q
    q = [0, 0, 0]
    q[0] = unwrap(psi1 - alpha1)
    q[1] = unwrap(psi2 - alpha2 - 2 * math.pi / 3)
    q[2] = unwrap(psi3 - alpha3 - 4 * math.pi / 3)

    # Kiểm tra xem điểm có nằm ngoài tầm với không
    out_of_reach = 0  # initialize with 0
    if math.isnan(q[0]) or math.isnan(q[1]) or math.isnan(q[2]):
        out_of_reach = 1

    if not out_of_reach:
        new_angle = [0, 0, 0]
        for i in range(3):
            target[i] = q[i] - home_offset_angle[i]
            target[i] = math.degrees(target[i])
            # Thực hiện interpolation và các bước khác nếu cần
            # ISR_Servo.setPulseWidth(servoIndex[i], microsec_now) có thể được thêm vào đây nếu cần thiết
    else:
        print("OUT OF REACH")
matplotlib.use("TkAgg")
def circular_trajectory(radius, num_points):
    """Tạo tọa độ (x, y) theo quỹ đạo đường tròn"""
    angle_now=0
    for i in range(0,num_points+3):
        angle = (2 * math.pi / num_points) * i
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        angle_deg=angle*180.0/math.pi
        # if(angle_deg<40):
        #     send_coordinates(x, y,0)
        # elif(angle_deg>40 and angle_deg<140):
        #     angle_now+=(2 * 180.0 / num_points)*0.1
        #     send_coordinates(x,y,angle_now)
        # else:
        #     send_coordinates(x, y, angle_now)
def create_circle(center, radius):
    return Point(center).buffer(radius)
def trapezoidal_profile_circle(radius, total_time, num_points, accel_time_ratio=0.3):
    # Tạo thời gian tương ứng cho mỗi pha (tăng tốc, giữ đều, giảm tốc)
    accel_time = total_time * accel_time_ratio  # Thời gian tăng tốc
    steady_time = total_time - 2 * accel_time  # Thời gian giữ đều
    time_steps = np.linspace(0, total_time, num_points)  # Các bước thời gian

    # Tạo profile vận tốc hình thang (tăng tốc, giữ đều, giảm tốc)
    velocities = np.zeros_like(time_steps)

    for i, t in enumerate(time_steps):
        if t < accel_time:
            velocities[i] = t / accel_time  # Tăng tốc
        elif t < accel_time + steady_time:
            velocities[i] = 1  # Giữ vận tốc đều
        else:
            velocities[i] = (total_time - t) / accel_time  # Giảm tốc

    # Tổng tích lũy góc quét phải là 2*pi
    total_angle = 2 * np.pi  # Tổng góc cần quét (360 độ)
    cumulative_angle = np.zeros(num_points)

    for i in range(1, num_points):
        cumulative_angle[i] = cumulative_angle[i - 1] + (velocities[i] / np.sum(velocities)) * total_angle

    # Tính tọa độ x, y cho quỹ đạo hình tròn dựa trên góc tích lũy
    x_positions = radius * np.cos(cumulative_angle)
    y_positions = radius * np.sin(cumulative_angle)

    return x_positions, y_positions, velocities
class TrajectoryApp:
    def __init__(self, root):
        self.velocity=[]
        self.root = root
        self.speed=25
        self.angle=40
        self.radius = 50 
        # self.ser = serial.Serial('COM10', 57600)  # Thay 'COM1' bằng cổng Serial của bạn và baudrate phù hợp
        self.root.title("Trajectory Tracker")
        # print("x check")
        # for i in range(-50,80,5):
        #     InvKinRRR(i,0,math.radians(self.angle))
        # for i in range(-50,50,5):
        #     InvKinRRR(0,i,math.radians(self.angle))
        InvKinRRR(0, 0, math.radians(self.angle))
        self.send_serial(target[0], target[1], target[2])
        time.sleep(0.1)
        self.set_speed(self.speed)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        # Khung nhập liệu cho x và y
        input_frame = ttk.Frame(root, padding="10")
        input_frame.grid(row=0, column=0, sticky="W")
        self.clear_button = ttk.Button(input_frame, text="Clear", command=self.clear_plot)
        self.clear_button.grid(row=0, column=10, padx=10, pady=5)
        # Nhãn và ô nhập cho x
        ttk.Label(input_frame, text="X:").grid(row=0, column=0, padx=5, pady=5, sticky="E")
        self.x_entry = ttk.Entry(input_frame, width=10)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)

        # Nhãn và ô nhập cho y
        ttk.Label(input_frame, text="Y:").grid(row=0, column=2, padx=5, pady=5, sticky="E")
        self.y_entry = ttk.Entry(input_frame, width=10)
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)
        self.radius_entry = ttk.Entry(input_frame, width=10)
        ttk.Label(input_frame, text="Radius:").grid(row=0, column=4, padx=5, pady=5, sticky="E")
        self.radius_entry.grid(row=0, column=5, padx=5, pady=5)
        # Nút Start
        self.start_button = ttk.Button(input_frame, text="Start", command=self.start_tracking)
        self.start_button.grid(row=0, column=6, padx=10, pady=5)
        self.square=0
        self.circle=0
        # Nút Load Trajectory
        self.load_button = ttk.Button(input_frame, text="Load circle", command=self.load_circle)
        self.load_button2 = ttk.Button(input_frame, text="Load SQUARE", command=self.load_square)
        self.load_button3 = ttk.Button(input_frame, text="Load Eclyp", command=self.load_eclyp)
        self.load_button.grid(row=0, column=7, padx=10, pady=5)
        self.load_button2.grid(row=0, column=8, padx=10, pady=5)
        self.load_button3.grid(row=0, column=9, padx=10, pady=5)
        angle_frame = ttk.Frame(root, padding="10")
        angle_frame.grid(row=2, column=0, sticky="W")
        ttk.Label(angle_frame, text="Current X:").grid(row=3, column=0, padx=5, pady=5, sticky="E")
        self.current_x_label = ttk.Label(angle_frame, text="0.0")
        self.current_x_label.grid(row=3, column=1, padx=5, pady=5)

        ttk.Label(angle_frame, text="Current Y:").grid(row=4, column=0, padx=5, pady=5, sticky="E")
        self.current_y_label = ttk.Label(angle_frame, text="0.0")
        self.current_y_label.grid(row=4, column=1, padx=5, pady=5)
    
        # Khung biểu đồ
        self.figure = plt.Figure(figsize=(12, 6))
        
        # Biểu đồ quỹ đạo động
        self.ax_dynamic = self.figure.add_subplot(121)
        self.ax_dynamic.set_title("Dynamic Trajectory")
        self.ax_dynamic.set_xlabel("X")
        self.ax_dynamic.set_ylabel("Y")
        self.ax_dynamic.grid(True)
        self.line_dynamic, = self.ax_dynamic.plot([], [], marker='o', markersize=5, color='blue')

        self.all_trajectory_x = []
        self.all_trajectory_y = []
        # Biểu đồ quỹ đạo từ file
        self.ax_loaded = self.figure.add_subplot(122)
        self.ax_loaded.set_title("Loaded Trajectory")
        self.ax_loaded.set_xlabel("X")
        self.ax_loaded.set_ylabel("Y")
        self.ax_loaded.grid(True)
        self.line_loaded, = self.ax_loaded.plot([], [], marker='x', markersize=5, color='red')

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
    def update_current_position(self, x, y):
        """Cập nhật tọa độ hiện tại"""
        self.current_x_label.config(text=f"{x:.2f}")
        self.current_y_label.config(text=f"{y:.2f}")
    def send_serial(self,x,y,theta):
        cmd_send = f"{x}/{y}#{theta};"
        print(cmd_send)
        # self.ser.write(cmd_send.encode())
    def set_speed(self,sp):
        message = f"{sp};"
        # print(f"Set Speed: {sp}")
        # self.ser.write(message.encode())
    def generate_straight_line_trajectory(self,x_start, y_start, x_target, y_target, num_points=40):
        """
        Tạo quỹ đạo thẳng với vận tốc hình thang từ điểm (x_start, y_start) đến (x_target, y_target).
        Vận tốc ban đầu tăng dần, sau đó giữ ổn định, và giảm dần trước khi tới đích.
        """

        # Tính toán sự thay đổi về tọa độ (delta)
        dx = x_target - x_start
        dy = y_target - y_start

        # Tính khoảng cách giữa hai điểm
        distance = math.sqrt(dx ** 2 + dy ** 2)
        total_distance=distance
        # Tính toán các tỷ lệ của quỹ đạo hình thang
        if distance == 0:
            return [x_start], [y_start]

        # Chia quá trình vận tốc hình thang thành 3 giai đoạn
        accel_distance = total_distance * 0.25  # 25% của tổng quãng đường để tăng tốc
        decel_distance = total_distance * 0.25  # 25% của tổng quãng đường để giảm tốc
        steady_distance = total_distance - accel_distance - decel_distance  # Phần còn lại giữ vận tốc ổn định

        self.trajectory_x = []
        self.trajectory_y = []

        # Giai đoạn tăng tốc
        for i in range(num_points // 3):
            t = i / (num_points // 3)  # Tham số thời gian cho tăng tốc
            t_squared = t ** 2
            cur_x = x_start + (t_squared * dx * (accel_distance / distance))
            cur_y = y_start + (t_squared * dy * (accel_distance / distance))
            self.trajectory_x.append(cur_x)
            self.trajectory_y.append(cur_y)

        # Giai đoạn giữ vận tốc ổn định
        for i in range(num_points // 3, 2 * num_points // 3):
            t = (i - num_points // 3) / (num_points // 3)
            cur_x = x_start + (accel_distance / distance + t * steady_distance / distance) * dx
            cur_y = y_start + (accel_distance / distance + t * steady_distance / distance) * dy
            self.trajectory_x.append(cur_x)
            self.trajectory_y.append(cur_y)

        # Giai đoạn giảm tốc
        for i in range(2 * num_points // 3, num_points):
            t = (i - 2 * num_points // 3) / (num_points // 3)
            t_squared = t ** 2
            cur_x = x_target - (1 - t_squared) * dx * (decel_distance / distance)
            cur_y = y_target - (1 - t_squared) * dy * (decel_distance / distance)
            self.trajectory_x.append(cur_x)
            self.trajectory_y.append(cur_y)
    def start_tracking(self):
        # Lấy giá trị x và y từ ô nhập liệu
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
        except ValueError:
            messagebox.showerror("Input Error", "Vui lòng nhập giá trị số hợp lệ cho X và Y.")
            return

        with self.lock:
            self.trajectory_x = []
            self.trajectory_y = []
            if(self.square==0 and self.circle==0):
                self.target_x = x
                self.target_y = y

                self.generate_straight_line_trajectory(self.x, self.y, self.target_x, self.target_y)
                self.x=self.target_x
                self.y=self.target_y
            else:
                self.trajectory_x = self.loaded_trajectory_x
                self.trajectory_y = self.loaded_trajectory_y
                # self.velocity = velocities

        self.start_button.config(state='disabled')
        self.stop_button = ttk.Button(self.root, text="Stop", command=self.stop_tracking)
        self.stop_button.grid(row=0, column=11, padx=10, pady=5)

        self.tracking = True
        self.thread = threading.Thread(target=self.update_position)
        self.thread.start()
    def stop_tracking(self):
        self.square=0
        self.circle=0
        self.tracking = False

        # self.send_serial(self.x,self.y,40)
        self.start_button.config(state='normal')
        self.load_button.config(state='normal')
        self.stop_button.destroy()

    def update_position(self):
        for i, (x, y) in enumerate(zip(self.trajectory_x, self.trajectory_y)):
            self.x = x
            self.y = y
            self.target_x = self.x
            self.target_y = self.y
            if not self.tracking:
                break
            print(x)
            InvKinRRR(x,y,math.radians(self.angle))
            self.send_serial(target[0], target[1], target[2])  # Giả sử góc luôn là 0
            self.plot_dynamic_trajectory(self.trajectory_x[:i + 1], self.trajectory_y[:i + 1])
            time.sleep(0.05)  # Simulate time delay
        self.stop_tracking()
            # Cập nhật biểu đồ trên giao diện Tkinter phải được thực hiện trong luồng chính



    def clear_plot(self):

        self.square=0
        self.circle=0
        # Xóa toàn bộ quỹ đạo đã lưu và làm mới biểu đồ
        with self.lock:
            self.trajectory_x = []
            self.trajectory_y = []
            self.loaded_trajectory_x=[]
            self.loaded_trajectory_y=[]
        self.ax_dynamic.cla()
        self.ax_dynamic.set_title("Dynamic Trajectory")
        self.ax_dynamic.set_xlabel("X")
        self.ax_dynamic.set_ylabel("Y")
        self.ax_dynamic.grid(True)
        self.ax_loaded.cla()
        self.ax_loaded.set_title("Loaded Trajectory")
        self.ax_loaded.set_xlabel("X")
        self.ax_loaded.set_ylabel("Y")
        self.ax_loaded.grid(True)
        self.canvas.draw()
        self.canvas.draw()

    def draw_workspace(self):
        K = 45
        l1 = 142
        l2 = 120
        R = 0
        xA = K * np.cos(np.pi / 6)
        yA = K * np.sin(np.pi / 6)
        p1 = create_circle((b1x, b1y), l1 + l2 + R)
        p2 = create_circle((-xA, -yA), abs(l1 - l2 - R))
        p3 = create_circle((b3x, b3y), l1 + l2 + R)
        p4 = create_circle((xA, -yA), abs(l1 - l2 - R))
        p5 = create_circle((b2x, b2y), l1 + l2 + R)
        p6 = create_circle((0, K), abs(l1 - l2 - R))
        # Tìm giao và trừ các hình tròn để tính không gian làm việc chính xác
        workspace = p1.intersection(p3).intersection(p5)
        workspace = workspace.difference(p2).difference(p4).difference(p6)
        workspace_x,workspace_y=workspace.exterior.xy
        # Vẽ workspace lên biểu đồ ax_loaded
        x_min=-45
        x_max=50
        y_min=-50
        y_max=35
        x=[x_min,x_min]
        y=[-150,150]
        self.ax_loaded.plot(workspace_x, workspace_y, 'g--', label='Workspace', alpha=0.5)
        self.ax_loaded.plot(x,y, color='r', linestyle='-', label='x = -45')  # Đường thẳng tại x = -45
        x=[x_max,x_max]
        y=[-150,150]
        self.ax_loaded.plot(x,y, color='r', linestyle='-')  # Đường thẳng tại x = -45
        x=[-100,100]
        y=[y_min,y_min]
        self.ax_loaded.plot(x,y, color='r', linestyle='-')  # Đường thẳng tại x = -45
        x=[-100,100]
        y=[y_max,y_max]
        self.ax_loaded.plot(x,y, color='r', linestyle='-')  # Đường thẳng tại x = -45
        # self.ax_loaded.plot(x,workspace_y, color='r', linestyle='-', label='x = 50')  # Đường thẳng tại x = -45
        # self.ax_loaded.axvline(x=50, color='b', linestyle='-', label='x = 50')    # Đường thẳng tại x = 50
        # self.ax_loaded.axhline(y=-50, color='m', linestyle='-', label='y = -50')  # Đường thẳng tại y = -50
        # self.ax_loaded.axhline(y=35, color='c', linestyle='-', label='y = 35')    # Đường thẳng tại y = 35

    def plot_dynamic_trajectory(self, x_data, y_data):
        """Vẽ lại quỹ đạo động, không xóa quỹ đạo cũ."""
        # self.ax_dynamic.cla()

        self.ax_dynamic.set_title("Dynamic Trajectory")
        self.ax_dynamic.set_xlabel("X")
        self.ax_dynamic.set_ylabel("Y")

        self.ax_dynamic.grid(True)
        self.ax_dynamic.plot(x_data, y_data, marker='o', markersize=5, color='blue')
        self.canvas.draw()
    def load_square(self):
        self.square=1
        self.circle=0
        num_points = 20
        try:
            x_now = float(self.x_entry.get())
            y_now = float(self.y_entry.get())
        except ValueError:
            messagebox.showerror("Input Error", "Vui lòng nhập giá trị số hợp lệ cho X và Y.")
            return
        x = []
        y = []
        x_append=x_now/2.0
        y_append=y_now/2.0
        for i in range(0,num_points):
            x.append(x_append)
            y.append(y_append)
            x_append=x_append-(x_now/num_points)
        for i in range(0,num_points):
            x.append(x_append)
            y.append(y_append)
            y_append=y_append-(y_now/num_points)
        for i in range(0,num_points):
            x.append(x_append)
            y.append(y_append)
            x_append=x_append+(x_now/num_points)
        for i in range(0,num_points):
            x.append(x_append)
            y.append(y_append)
            y_append=y_append+(y_now/num_points)
        # loaded_x, loaded_y, velocities = trapezoidal_profile_circle(15, 10, num_points)  # radius time point
        with self.lock:
            self.loaded_trajectory_x = x
            self.loaded_trajectory_y = y

            # self.velocity = velocities
        # Vẽ quỹ đạo đã tải trên biểu đồ thứ hai
        self.ax_loaded.cla()
        self.ax_loaded.set_title("Loaded Trajectory")
        self.ax_loaded.set_xlabel("X")
        self.ax_loaded.set_ylabel("Y")
        self.ax_loaded.grid(True)
        self.ax_loaded.plot(self.loaded_trajectory_x, self.loaded_trajectory_y, '-',
                            color='red')
        self.draw_workspace()
        self.canvas.draw()
    def load_eclyp(self):
        try:
            radius = float(self.radius_entry.get())
        except ValueError:
            messagebox.showerror("Input Error", "Radius not filled")
            return
        k=2.1
        self.square=0
        self.circle=1
        num_points=100
        # self.angle=0
        x = []
        y = []
        for i in np.arange(0,44*math.pi,math.pi/1440):
            px =  radius*(k + 1)*math.cos(i) - radius*math.cos((k + 1)*i)
            py = radius*(k + 1)*math.sin(i) - radius*math.sin((k + 1)*i)
            x.append(px)
            y.append(py)
        # loaded_x, loaded_y, velocities = trapezoidal_profile_circle(10, 10, num_points) #radius time point
        with self.lock:
            self.loaded_trajectory_x = x
            self.loaded_trajectory_y = y
            # self.velocity=velocities
        # Vẽ quỹ đạo đã tải trên biểu đồ thứ hai
        self.ax_loaded.cla()
        self.ax_loaded.set_title("Loaded Trajectory")
        self.ax_loaded.set_xlabel("X")
        self.ax_loaded.set_ylabel("Y")
        self.ax_loaded.grid(True)
        self.ax_loaded.plot(self.loaded_trajectory_x, self.loaded_trajectory_y, '-',
                            color='red')
        self.draw_workspace()
        self.canvas.draw()
    def load_circle(self):
        # circular_trajectory()
        try:
            radius = float(self.radius_entry.get())
        except ValueError:
            messagebox.showerror("Input Error", "Radius not filled")
            return
        self.square=0
        self.circle=1
        num_points=100
        # self.angle=0
        x = []
        y = []
        for i in range(num_points):
            angle = (2 * math.pi / num_points) * i
            px = radius * math.cos(angle)
            py = radius * math.sin(angle)
            x.append(px)
            y.append(py)
        # loaded_x, loaded_y, velocities = trapezoidal_profile_circle(10, 10, num_points) #radius time point
        with self.lock:
            self.loaded_trajectory_x = x
            self.loaded_trajectory_y = y
            # self.velocity=velocities
        # Vẽ quỹ đạo đã tải trên biểu đồ thứ hai
        self.ax_loaded.cla()
        self.ax_loaded.set_title("Loaded Trajectory")
        self.ax_loaded.set_xlabel("X")
        self.ax_loaded.set_ylabel("Y")
        self.ax_loaded.grid(True)
        self.ax_loaded.plot(self.loaded_trajectory_x, self.loaded_trajectory_y, '-',
                            color='red')
        self.draw_workspace()
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
