from pynput import keyboard
import threading
import time

# Biến toàn cục lưu giá trị phím nhấn
pressed_key = None

# Hàm xử lý khi phím được nhấn
def on_press(key):
    global pressed_key
    try:
        pressed_key = key.char
    except AttributeError:
        pressed_key = str(key)

# Hàm xử lý khi phím được thả ra
def on_release(key):
    global pressed_key
    if key == keyboard.Key.esc:
        # Dừng listener
        return False

# Hàm in giá trị phím nhấn sau mỗi 1 giây
def print_pressed_key():
    while True:
        if pressed_key:
            print(f'Pressed key: {pressed_key}')
        time.sleep(1)

# Tạo và chạy listener trong một thread riêng
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Chạy hàm in giá trị phím nhấn trong thread chính
print_pressed_key()
