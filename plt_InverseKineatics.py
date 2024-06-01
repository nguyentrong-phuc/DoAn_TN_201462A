import math as m
import numpy as np
import matplotlib.pyplot as plt
l1=46.4 #mm
l2=100 #mm
l3=100 #mm
Right = 1
Left = 2
def CAL_Inverse_Kinematics(Leg,x,y,z):
    if(Leg == Right):
        L1 = -l1
    elif(Leg == Left):
        L1 = l1
    # Theta1
    alpha = m.acos(y/np.sqrt(y**2 + z**2))
    t1 = m.acos(L1/np.sqrt(z**2 + y**2)) - alpha
    t1 = -t1
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    # Theta3
    A = - x
    if(t1!=0):
        B = (L1*c1 - y)/s1
    else:
        B = (L1*s1 - z)/c1
    c3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
    s3 = np.sqrt(m.fabs(1 - c3**2))
    t3 = m.atan2(s3, c3)
    t3=(L1/l1)*t3
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  # /((l2+l3*c3)**2 + (l3*s3)**2)
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  # /((l2+l3*c3)**2 + (l3*s3)**2)
    t2 = m.atan2(s2, c2)
    t2=(L1/l1)*t2 
    # Forward kinematics
    Px = (L1/l1)*(l3*np.sin(t2 - t3) + l2*np.sin(t2))
    Py = -np.sin(t1)*(l3 * np.cos(t2 - t3) + l2*np.cos(t2)) + L1*np.cos(t1)
    Pz = -np.cos(t1)*(l3 * np.cos(t2 - t3) + l2*np.cos(t2)) - L1*np.sin(t1)
    return t1,t2,t3,Px,Py,Pz

ax = plt.axes(projection="3d")
fig = plt.figure(figsize=(10, 10))
a=np.arange(0,2,0.05)
for t in a:
    # Phương trình tham số quỹ đạo theo  thời gian t
    if(t<=1):
        x = -60*t+30
        y = l1 + 10*t
        z = -100
    else:
        alpha= np.pi*(2-t)
        x = 30*np.cos(alpha)
        y = l1 + 10*(2-t)
        z = -100+30*np.sin(alpha)
    # Kiểm tra điều kiện không gian hoạt động
    k1 = x**2 + y**2 + z**2
    k2 = y**2 + z**2
    if( k2< l1**2 or k1> (l1**2 + (l2 + l3)**2)):
        print(x, y, z)
        print("Out of workspace")
        break
    t1,t2,t3,Px,Py,Pz = CAL_Inverse_Kinematics(Left,x,y,z)
    # Vẽ đồ thị mô phỏng quỹ đạo
    ax.scatter3D(Px, Py, Pz )#color="green")
    plt.subplot(2, 2, 1)
    plt.plot(t, t1*180/m.pi, 'r.')
    plt.title('theta1')
    plt.subplot(2, 2, 2)
    plt.plot(t, t2*180/m.pi, 'm.')
    plt.title('theta2')
    plt.subplot(2, 2, 3)
    plt.plot(t, t3*180/m.pi, 'c.')
    plt.title('theta3')

    plt.subplot(2, 2, 4)
    plt.plot(t, t1 * 180 / m.pi, 'r.')
    plt.plot(t, t2 * 180 / m.pi, 'm.')
    plt.plot(t, t3 * 180 / m.pi, 'c.')
    plt.title('THETA')
plt.show()
