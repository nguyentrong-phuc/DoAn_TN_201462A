from pynput import keyboard
import threading
import time

# Biến toàn cục lưu giá trị phím nhấn
pressed_key = None
count = 0
flag = 0
Pre_presskey = None

# Hàm xử lý khi phím được nhấn
def on_press(key):
    global pressed_key, Pre_presskey, flag, count
    try:
        test_key = key.char
        if(flag == 0):
            pressed_key = key.char
            Pre_presskey = pressed_key
    except AttributeError:
        test_key = str(key)
        if key == keyboard.Key.esc:
            pressed_key = str(key)
            print("Stoping")
            if(Pre_presskey == 'w'):
                print("endTrot(Fw)")
                print("allToInitial(Fw)")
            elif (Pre_presskey == 's'):
                print("endTrot(Bw)")
                print("allToInitial(Bw)")
            elif (Pre_presskey == 'a'):
                print("endTrot(L)")
                print("allToInitial(L)")
            elif (Pre_presskey == 'd'):
                print("endTrot(R)")
                print("allToInitial(R)")
            Pre_presskey = None
            flag = 0
            count = 0

def move_robot():
    global count, flag
    while True:
        # print(f'Pressed key: {pressed_key}')
        # time.sleep(1)
        if (pressed_key == 'w'):
            flag = 1
            if(count == 0):
                print("--------------------")
                print(f'Robot is moving: Forward')
                print("allMove(Fw)")
                print("startTrot(Fw)")
                count = 1
            elif(count == 1):
                print("Trotgait(Fw)")
            time.sleep(1)

        elif (pressed_key == 's'):
            flag = 1
            if (count == 0):
                print("--------------------")
                print(f'Robot is moving: Backward')
                print("allMove(Bw)")
                print("startTrot(Bw)")
                count = 1
            elif (count == 1):
                print("Trotgait(Bw)")
            time.sleep(1)

        elif (pressed_key == 'a'):
            flag = 1
            if (count == 0):
                print("--------------------")
                print(f'Robot is moving: Left')
                print("allMove(L)")
                print("startTrot(L)")
                count = 1
            elif (count == 1):
                print("Trotgait(L)")
            time.sleep(1)

        elif (pressed_key == 'd'):
            flag = 1
            if (count == 0):
                print("--------------------")
                print(f'Robot is moving: Right')
                print("allMove(R)")
                print("startTrot(R)")
                count = 1
            elif (count == 1):
                print("Trotgait(R)")
            time.sleep(1)

# Tạo và chạy listener trong một thread riêng
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Chạy hàm in giá trị phím nhấn trong thread chính
move_robot()
