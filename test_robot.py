from robot import Motor
import unittest
import unittest.mock as mock
from unittest.mock import Mock


class TestMotorClass(unittest.TestCase):


    def test_set_zero_cur_position(self):
        serial_mock = Mock()
        motor = Motor(0x01, serial_mock)

        # Simulate correct respond from motor
        serial_mock.read.return_value = bytearray([0]*26)      

        # Call to test function
        motor.set_zero_cur_position()

        # Test if set_zero_cur_position(...) pass to serial port correct command
        serial_mock.write.assert_called_once_with(b'>\x19\x01\x00X')

if __name__ == '__main__':
    unittest.main()
