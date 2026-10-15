
import time
from rov_mqtt.controller import PS4Controller

controller = PS4Controller()

try:
    while True:
        thr, turn = controller.read_drive()
        print(f"thr={thr:+.2f} | turn={turn:+.2f}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Controller test stopped.")

finally:
    controller.close()
