import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.ops import unary_union

# Hàm vẽ hình tròn
def draw_circle(ax, center, radius, color):
    circle = plt.Circle(center, radius, color=color, fill=False, linewidth=2)
    ax.add_artist(circle)

# Tính thông số từ đề bài
K = 331 / np.sqrt(3)
l1 = 166
l2 = 110
R = 75.06

# Vị trí của các actuator
xA = K * np.cos(np.pi/6)
yA = K * np.sin(np.pi/6)

# Tạo figure để vẽ workspace
fig, ax = plt.subplots()

# Vẽ các hình tròn (xấp xỉ không gian làm việc)
draw_circle(ax, (-xA, -yA), l1 + l2 + R, 'r')  # Circle around first actuator
draw_circle(ax, (xA, -yA), l1 + l2 + R, 'b')   # Circle around second actuator
draw_circle(ax, (0, K), l1 + l2 + R, 'g')      # Circle around third actuator
draw_circle(ax, (-xA, -yA), abs(l1 - l2 - R), 'r')  # Smaller circle first actuator
draw_circle(ax, (xA, -yA), abs(l1 - l2 - R), 'b')   # Smaller circle second actuator
draw_circle(ax, (0, K), abs(l1 - l2 - R), 'g')      # Smaller circle third actuator

# Thêm các điểm gốc và điểm trung tâm
ax.plot(0, 0, 'k+', markersize=10)
ax.plot(-xA, -yA, 'xr', markersize=10)
ax.plot(xA, -yA, 'xb', markersize=10)
ax.plot(0, K, 'xg', markersize=10)

# Thêm các ghi chú
ax.text(10, 10, '(0,0)', fontsize=12)
ax.text(-xA - 60, -yA - 10, 'A1', fontsize=12)
ax.text(xA + 20, -yA - 10, 'A2', fontsize=12)
ax.text(0, K + 40, 'A3', fontsize=12)

# Giới hạn vùng hiển thị
ax.set_xlim([-420, 420])
ax.set_ylim([-420, 420])
ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_aspect('equal')
ax.legend(['1st kinematic chain', '2nd kinematic chain', '3rd kinematic chain'])
ax.set_title("Workspace Approximation based on Kinematic Lengths")

plt.show()

# Phần dưới để tính toán workspace chính xác sử dụng shapely (giao và trừ các hình tròn)
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

# Hàm tạo hình tròn (polyshape)
def create_circle(center, radius):
    return Point(center).buffer(radius)

# Tạo các hình tròn polyshape
p1 = create_circle((-xA, -yA), l1 + l2 + R)
p2 = create_circle((-xA, -yA), abs(l1 - l2 - R))
p3 = create_circle((xA, -yA), l1 + l2 + R)
p4 = create_circle((xA, -yA), abs(l1 - l2 - R))
p5 = create_circle((0, K), l1 + l2 + R)
p6 = create_circle((0, K), abs(l1 - l2 - R))

# Tìm giao và trừ các hình tròn để tính không gian làm việc chính xác
workspace = p1.intersection(p3).intersection(p5)
workspace = workspace.difference(p2).difference(p4).difference(p6)

# Vẽ workspace sử dụng shapely
fig, ax = plt.subplots()

# Vẽ workspace chính xác
x, y = workspace.exterior.xy
ax.plot(x, y, 'k')

# Thêm các điểm và ghi chú
ax.plot(0, 0, 'k+', markersize=10)
ax.text(10, 10, '(0,0)', fontsize=12)

# Giới hạn vùng hiển thị
ax.set_xlim([-220, 220])
ax.set_ylim([-220, 220])
ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_aspect('equal')
ax.set_title("Approximated Workspace")

plt.show()
