import lgpio
import time
print("Attempting to open GPIO chip 0...")
try:
    h = lgpio.gpiochip_open(0)
    print("SUCCESS: GPIO chip opened!")
    lgpio.gpiochip_close(h)
except Exception as e:
    print(f"FAILURE: Cannot access hardware. Error: {e}")
