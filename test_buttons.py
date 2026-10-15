
import pygame
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected")
    pygame.quit()
    raise SystemExit

pad = pygame.joystick.Joystick(0)
pad.init()

previous = [0] * pad.get_numbuttons()

print("Controller:", pad.get_name())
print("Press one button at a time. Ctrl+C to stop.")

try:
    while True:
        pygame.event.pump()

        for i in range(pad.get_numbuttons()):
            current = pad.get_button(i)

            if current and not previous[i]:
                print(f"Button {i} PRESSED")

            previous[i] = current

        time.sleep(0.02)

except KeyboardInterrupt:
    print("Test stopped")

finally:
    pygame.quit()
