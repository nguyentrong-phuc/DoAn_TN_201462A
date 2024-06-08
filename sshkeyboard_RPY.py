from sshkeyboard import listen_keyboard

Roll= 0
Pitch= 0
Yaw= 0

def on_press(key):
    global Roll, Pitch, Yaw
    try:
        # print(f"Phím đã nhấn: {key}")  
        if (key == '1'):
            Roll += 1
        elif (key == '2'):
            Roll -= 1
        elif (key == '3'):
            Pitch += 1
        elif (key == '4'):
            Pitch -= 1
        elif (key == '5'):
            Yaw += 1
        elif (key == '6'):
            Yaw -= 1
        print('RollPitchYaw({},{},{})'.format(Roll, Pitch, Yaw))
        
    except AttributeError:
        print("Phím đặc biệt đã được nhấn")
    

listen_keyboard(on_press=on_press)
