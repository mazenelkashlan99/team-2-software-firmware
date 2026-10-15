import pygame

class PS4Controller:
    def __init__(self,deadzone: float = 0.12):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            pygame.quit()
            raise RuntimeError("No PS4 Controller detected")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.deadzone = deadzone

        button_count = self.joystick.get_numbuttons()
        print(
            f"[Controller] Connected: {self.joystick.get_name()}; "
            f"buttons={button_count}; "
            f"Triangle index 3 available={button_count > 3}; "
            f"OPTIONS index 6 available={button_count > 6}"
        )


    def _apply_deadzone(self,value:float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def read_drive(self) -> tuple[float,float]:
        try:
            pygame.event.pump()                                # Updating the last moves
            forward = -self.joystick.get_axis(1)               # Up on left stick is negative.

            turn = self.joystick.get_axis(0)

            thr = self._apply_deadzone(forward)
            turn = self._apply_deadzone(turn)

            return thr, turn
        except pygame.error as error:
            raise RuntimeError("PS4 controller disconnected") from error

    def read_buttons(self) -> tuple[bool,...]:
        try:
            pygame.event.pump()
            return tuple(
                bool(self.joystick.get_button(i))
                for i in range(self.joystick.get_numbuttons())
            )
        except pygame.error as error:
            raise RuntimeError("PS4 controller disconnected") from error

    def close(self):
        pygame.quit()
