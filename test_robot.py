import serial
from robot import Motor
import unittest
import unittest.mock as mock


class TestMotorClass(unittest.TestCase):

    @mock.patch("serial.Serial")
    def test_set_zero_cur_position(self, mock_serial):
        motor = Motor(0x01, mock_serial)

        mock_serial.read.return_value = bytearray([0]*26)
        
        motor.set_zero_cur_position()
        mock_serial.write.assert_called_once_with(b'01020304')
        
        self.assertTrue(mock_serial.write.called, 'Serial write method not called')

if __name__ == '__main__':
    unittest.main()
