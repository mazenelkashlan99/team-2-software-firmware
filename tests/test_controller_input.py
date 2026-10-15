import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rov_mqtt.controller import PS4Controller


class PygameControllerInputTests(unittest.TestCase):
    def test_read_buttons_preserves_triangle_and_options_indices(self):
        joystick = Mock()
        joystick.get_name.return_value = "test PS4 controller"
        joystick.get_numbuttons.return_value = 8
        joystick.get_button.side_effect = lambda index: index in {3, 6}

        with (
            patch("rov_mqtt.controller.pygame.init"),
            patch("rov_mqtt.controller.pygame.quit"),
            patch("rov_mqtt.controller.pygame.joystick.init"),
            patch("rov_mqtt.controller.pygame.joystick.get_count", return_value=1),
            patch("rov_mqtt.controller.pygame.joystick.Joystick", return_value=joystick),
            patch("rov_mqtt.controller.pygame.event.pump"),
        ):
            controller = PS4Controller()
            buttons = controller.read_buttons()
            controller.close()

        self.assertEqual(len(buttons), 8)
        self.assertTrue(buttons[3])  # Triangle / Reset ESTOP
        self.assertTrue(buttons[6])  # OPTIONS / ESTOP
        self.assertEqual(
            [call.args[0] for call in joystick.get_button.call_args_list],
            list(range(8)),
        )


if __name__ == "__main__":
    unittest.main()
