import math
import numpy as np
rb = 0.185
rp_=0.045
rp = [0.045, 0.045, 0.045]
L1 = 0.142
L2 = 0.12
home_offset_angle = [0.0, 0.0, 0.0]
b1x = rb * math.cos(-math.pi / 6)
b1y = rb * math.sin(-math.pi / 6)
b2x = 0
b2y = rb
b3x = rb * math.cos(7 * math.pi / 6)
b3y = rb * math.sin(7 * math.pi / 6)

a_angles = [-math.pi / 6, math.pi / 2, 7 * math.pi / 6]
# Hàm unwrap để điều chỉnh góc theta
def unwrap(theta):
    if theta < 0:
        theta += 2 * math.pi
    return theta

# Hàm chuyển đổi từ radian sang độ
def rad_to_deg(theta):
    return theta * 180 / math.pi
def ForwardRRR(q0,q1,q2,theta,x,y):
    e1x = b1x + L1 * np.cos(q0)
    e1y = b1y + L1 * np.sin(q0)

    e2x = b2x + L1 * np.cos(q1)
    e2y = b2y + L1 * np.sin(q1)

    e3x = b3x + L1 * np.cos(q2)
    e3y = b3y + L1 * np.sin(q2)
    le1=((e1x-x)**2+(e1y-y)**2)
    Le1=(L2**2+rp_**2)
    print(le1)
    print(Le1)
    theta=math.acos((le1-Le1)/(2*L2*rp_))
    print(rad_to_deg(theta))
    print(rad_to_deg(q0+a_angles[0]))
    Le2=(L2**2+rp_**2-2*L2*rp_*math.cos(theta-a_angles[1]))
    Le3=(L2**2+rp_**2-2*L2*rp_*math.cos(theta-a_angles[2]))
    # print(Le1)
    # print(Le2)
    G1 = (2 * e1x) - (2 * e2x)
    G2 = (2 * e1y) - (2 * e2y)
    G3 = (e2x**2 + e2y**2) - (e1x**2 + e1y**2)

    G4 = (2 * e1x) - (2 * e3x)
    G5 = (2 * e1y) - (2 * e3y)
    G6 = (e3x**2 + e3y**2) - (e1x**2 + e1y**2)

    # Ma trận V và vector W
    V = np.array([[G1, G2], [G4, G5]])
    W = np.array([Le2-Le1-G3,Le3-Le1-G6])   

    # Giải hệ phương trình để tìm tọa độ end-effector
    F = np.linalg.solve(V, W)
    cx, cy = F[0], F[1]
    return cx,cy
# Hàm Inverse Kinematics cho robot 3RRR
def InvKinRRR(px, py, theta):


    # Tọa độ các điểm b1, b2, b3


    # Tính toán tọa độ các điểm a1, a2, a3
    a1x = px + rp[0] * math.cos(theta + a_angles[0])
    a1y = py + rp[0] * math.sin(theta + a_angles[0])
    a2x = px + rp[1] * math.cos(theta + a_angles[1])
    a2y = py + rp[1] * math.sin(theta + a_angles[1])
    a3x = px + rp[2] * math.cos(theta + a_angles[2])
    a3y = py + rp[2] * math.sin(theta + a_angles[2])
    # Tính toán các khoảng cách a1b1, a2b2, a3b3
    a1b1 = math.sqrt((a1x - b1x) ** 2 + (a1y - b1y) ** 2)
    a2b2 = math.sqrt((a2x - b2x) ** 2 + (a2y - b2y) ** 2)
    a3b3 = math.sqrt((a3x - b3x) ** 2 + (a3y - b3y) ** 2)

    # Tính góc alpha1, alpha2, alpha3
    alpha1 = math.acos((a1b1 ** 2 + L1 ** 2 - L2 ** 2) / (2 * L1 * a1b1))
    alpha2 = math.acos((a2b2 ** 2 + L1 ** 2 - L2 ** 2) / (2 * L1 * a2b2))
    alpha3 = math.acos((a3b3 ** 2 + L1 ** 2 - L2 ** 2) / (2 * L1 * a3b3))

    # Tính toán các góc psi1, psi2, psi3
    psi1 = math.atan2(a1y - b1y, a1x - b1x)
    psi2 = math.atan2(a2y - b2y, a2x - b2x)
    psi3 = math.atan2(a3y - b3y, a3x - b3x)

    # Sử dụng hàm unwrap để điều chỉnh các góc psi
    psi1 = unwrap(psi1)
    psi2 = unwrap(psi2)
    psi3 = unwrap(psi3)

    # Tính toán các góc q1, q2, q3
    q = [0, 0, 0]
    new_q=[0,0,0]
    new_q[0] = unwrap(psi1 - alpha1)
    new_q[1] = unwrap(psi2 - alpha2)
    new_q[2] = unwrap(psi3 - alpha3)
    q[0] = unwrap(psi1 - alpha1)
    q[1] = unwrap(psi2 - alpha2 - (2 * math.pi / 3))
    q[2] = unwrap(psi3 - alpha3 - (4 * math.pi / 3))
    
    # Kiểm tra nếu bất kỳ góc nào là NaN
    out_of_reach = False
    if math.isnan(q[0]) or math.isnan(q[1]) or math.isnan(q[2]):
        out_of_reach = True
    return new_q
    if not out_of_reach:
        # Cập nhật góc target
        target = [0, 0, 0]
        for i in range(3):
            target[i] = q[i] - home_offset_angle[i]
            target[i] = rad_to_deg(target[i])
        # return target
    else:
        print("OUT OF REACH")
        return None

# Ví dụ sử dụng hàm InvKinRRR
px = 0.0
py = 0.0
theta = math.radians(0)
result = InvKinRRR(px, py, theta)
x,y=ForwardRRR(result[0],result[1],result[2],0,0,0)
# print(x)
# print(y)
if result is not None:
    print("Góc các servo:", result)
