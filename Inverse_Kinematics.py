def LEFT_Inverse_Kinematics(leg,x,y,z):
    global posFL, posRL, posFR, posRR
    #check work space
    Check_work_space(x,y,z)
    # Theta1
    alpha = m.acos(y/np.sqrt(y**2 + z**2))
    t1 = -(m.acos(l1/np.sqrt(z**2 + y**2)) - alpha)
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    # Theta3
    A = -x
    if(t1!=0):
        B = (l1*c1 - y)/s1
    else:
        B = (-l1*s1 - z)/c1
    c3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
    s3 = np.sqrt(m.fabs(1 - c3**2))
    t3 = m.atan2(s3, c3)
    t3 = t3*(L1/l1)
    
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
    t2 = m.atan2(s2, c2)
    t2 = t2*(L1/l1)

    # Convert Rad -> Deg
    t1 = t1*180/np.pi
    t2 = t2*180/np.pi
    t3 = t3*180/np.pi

    # Run and save new position
    if(leg == Rear):
        setLegAngles(rear_left,t1,t2,t3)
        posRL = [x, y, z] 
    elif(leg == Front):
        setLegAngles(front_left,t1,t2,t3)
        posFL = [x, y, z] 

def RIGHT_Inverse_Kinematics(leg,x,y,z):
    global posFL, posRL, posFR, posRR
    #check work space
    Check_work_space(x,y,z)
    # Theta1
    alpha = m.asin(y/np.sqrt(y**2 + z**2))
    t1 = -(m.asin(l1/np.sqrt(z**2 + y**2)) + alpha)
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    # Theta3
    A = -x
    if(t1!=0):
        B = (-l1*c1 - y)/s1
    else:
        B = (l1*s1 - z)/c1
    c3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
    s3 = np.sqrt(m.fabs(1 - c3**2))
    t3 = -m.atan2(s3, c3)
    # Kiem tra dk theta3
    if (t3 > 0):
        print("Error Theta3")
        exit()
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
    t2 = -m.atan2(s2, c2)
    
    # Convert Rad -> Deg
    t1 = t1*180/np.pi
    t2 = t2*180/np.pi
    t3 = t3*180/np.pi
    
    # Run and save new position
    if(leg == Rear):
        setLegAngles(rear_right,t1,t2,t3)
        posRR = [x, y, z] 
    elif(leg == Front):
        setLegAngles(front_right,t1,t2,t3)
        posFR = [x, y, z] 


#-------------------------------------------------------#

#Define leg adress
front_left = 0
front_right = 3
rear_left = 6
rear_right = 9

def Inverse_Kinematics(legAdress, x, y, z):
  global posFL, posRL, posFR, posRR
  if(legAdress == front_left or legAdress == rear_left):
    L1 = l1
  elif(legAdress == front_right or legAdress == rear_right):
    L1 = -l1
  else:
    print("Wrong adress...")
    exit()
  #check work space
  Check_work_space(x,y,z)
  # Theta1
  alpha = m.acos(y/np.sqrt(y**2 + z**2))
  t1 = -(m.acos(L1/np.sqrt(z**2 + y**2)) - alpha)
  c1 = np.cos(t1)
  s1 = np.sin(t1)
  # Theta3
  A = -x
  if(t1!=0):
      B = (L1*c1 - y)/s1
  else:
      B = (-L1*s1 - z)/c1
  c3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
  s3 = np.sqrt(m.fabs(1 - c3**2))
  t3 = m.atan2(s3, c3)
  # Theta2
  s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
  c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
  t2 = m.atan2(s2, c2)

  # Convert Rad -> Deg
  t1 = t1*180/np.pi
  t2 = t2*180/np.pi
  t3 = t3*180/np.pi

  # Run and save new position
  if(leg == rear_left):
      setLegAngles(rear_left,t1,t2,t3)
      posRL = [x, y, z] 
  elif(leg == front_left):
      setLegAngles(front_left,t1,t2,t3)
      posFL = [x, y, z] 
  elif(leg == rear_right):
      setLegAngles(rear_right,t1,t2,t3)
      posRR = [x, y, z] 
  elif(leg == front_right):
      setLegAngles(front_right,t1,t2,t3)
      posFR = [x, y, z]  
