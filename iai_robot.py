import serial
import time


class IAI:
    def __init__(self, set_port, set_baudrate, set_timeout):
        # start serial connection
        self.ser = serial.Serial(set_port, set_baudrate, timeout=set_timeout)

        # servo xyz on
        self.ser.write('!00232071b0\r\n'.encode())
        read = self.ser.readline().decode()
        print(read + '\n')
        time.sleep(1)

        # set home
        self.ser.write('!0023307000000a0\r\n'.encode())
        read = self.ser.readline().decode()
        print(read + '\n')
        time.sleep(15)

        self.origin = None
        self.workspace_boundary = None  # [x_min, y_min, x_max, y_max] in absolute coordinate

    def move(self, message_id, axis, acceleration, speed, x_target, y_target, z_target):
        """
        move robot
        example: move(ser,'relative','xz',0.3,100,50,50,0)
        """
        if message_id == 'absolute':
            message_id = '234'
        elif message_id == 'relative':
            message_id = '235'

        axis_pattern = 0b0
        if 'x' in axis:
            axis_pattern = axis_pattern + 0b1
        if 'y' in axis:
            axis_pattern = axis_pattern + 0b10
        if 'z' in axis:
            axis_pattern = axis_pattern + 0b100
        byte_format = 2
        byte_adding = byte_format - len(hex(axis_pattern).lstrip('0x'))
        adding_text = ''
        for i in range(0, byte_adding):
            adding_text = adding_text + '0'
        axis_pattern = adding_text + hex(axis_pattern).lstrip('0x')

        byte_format = 4
        byte_adding = byte_format - len(hex(int(acceleration * 100)).lstrip('0x'))
        adding_text = ''
        for i in range(0, byte_adding):
            adding_text = adding_text + '0'
        acceleration = adding_text + hex(int(acceleration * 100)).lstrip('0x')

        byte_format = 4
        byte_adding = byte_format - len(hex(int(speed)).lstrip('0x'))
        adding_text = ''
        for i in range(0, byte_adding):
            adding_text = adding_text + '0'
        speed = adding_text + hex(int(speed)).lstrip('0x')

        position = ''
        byte_format = 8
        if 'x' in axis:
            if x_target >= 0:
                byte_adding = byte_format - len(hex(int(x_target * 1000)).lstrip('0x'))
                adding_text = ''
                for i in range(0, byte_adding):
                    adding_text = adding_text + '0'
                position = position + adding_text + hex(int(x_target * 1000)).lstrip('0x')
            else:
                x_target = -x_target
                position = position + hex(int(16 ** byte_format - x_target * 1000)).lstrip('0x')
        if 'y' in axis:
            if y_target >= 0:
                byte_adding = byte_format - len(hex(int(y_target * 1000)).lstrip('0x'))
                adding_text = ''
                for i in range(0, byte_adding):
                    adding_text = adding_text + '0'
                position = position + adding_text + hex(int(y_target * 1000)).lstrip('0x')
            else:
                y_target = -y_target
                position = position + hex(int(16 ** byte_format - y_target * 1000)).lstrip('0x')
        if 'z' in axis:
            if z_target >= 0:
                byte_adding = byte_format - len(hex(int(z_target * 1000)).lstrip('0x'))
                adding_text = ''
                for i in range(0, byte_adding):
                    adding_text = adding_text + '0'
                position = position + adding_text + hex(int(z_target * 1000)).lstrip('0x')
            else:
                z_target = -z_target
                position = position + hex(int(16 ** byte_format - z_target * 1000)).lstrip('0x')

        string_command = '!00' + message_id + axis_pattern + acceleration + acceleration + speed + position

        checksum = 0
        for i in range(0, len(string_command)):
            checksum = checksum + ord(string_command[i])
        checksum = hex(int(checksum)).lstrip('0x')
        checksum = checksum[len(checksum) - 2:len(checksum)]

        string_command = string_command + checksum

        self.ser.write((string_command + '\r\n').encode())
        read = self.ser.readline().decode()
        print(read + '\n')

    def move_in_workspace(self, x, y):
        """
        move robot in workspace coordinate
        """
        # check constraints
        if not ((self.workspace_boundary[0] < x < self.workspace_boundary[1]) or (self.workspace_boundary[2] < y < self.workspace_boundary[3])):
            print("target position is out of workspace boundary")
            return False

        x_abs = self.origin[0] + x
        y_abs = self.origin[1] + y
        self.move('absolute', 'xy', 0.3, 100, x_abs, y_abs, 0)
        time.sleep(2)
        return True

    def drill(self):
        # run motor code here

        #############
        self.move('relative', 'z', 0.3, 100, 0, 0, 20)
        time.sleep(2)
        self.move('relative', 'z', 0.3, 100, 0, 0, -20)
        time.sleep(2)

        # stop motor code here

        ####################
    def set_workspace(self, origin, x_len, y_len):
        """
        set workspace range and return contraints to not go out the area
        :param top_left_pos: x, y positions ,in mm from home
        :param bottom_right_pos: x, y positions ,in mm from home
        :return: constraint x_min, x_max, y_min, y_max ,in mm from home
        """
        x_min, y_max = origin
        x_max, y_min = origin[0] + x_len, origin[1] - y_len
        # show workspace
        self.move('absolute', 'xyz', 0.3, 100, x_min, y_max, 0)
        time.sleep(2)
        self.move('absolute', 'xyz', 0.3, 100, x_min, y_min, 0)
        time.sleep(2)
        self.move('absolute', 'xyz', 0.3, 100, x_min, y_min, 0)
        time.sleep(2)
        self.move('absolute', 'xyz', 0.3, 100, x_min, y_max, 0)
        time.sleep(2)
        self.move('absolute', 'xyz', 0.3, 100, x_max, y_max, 0)
        time.sleep(2)

        self.workspace_boundary = [x_min, y_min, x_max, y_max]
        self.origin = origin

    def check_status(self, axis):
        """
        check servo status
        """
        message_id = '212'

        axis_pattern = 0b0
        if 'x' in axis:
            axis_pattern = axis_pattern + 0b1
        if 'y' in axis:
            axis_pattern = axis_pattern + 0b10
        if 'z' in axis:
            axis_pattern = axis_pattern + 0b100

        byte_format = 2
        axis_pattern = hex(axis_pattern).lstrip('0x').rjust(byte_format, '0')

        string_command = '!00' + message_id + axis_pattern
        cs = self.checksum(string_command)
        string_command += cs

        self.ser.write((string_command + '\r\n').encode())
        response = self.ser.readline().decode()
        print(response + '\n')

        # interpret response
        axis_pattern = response[7:9]
        axis_status = response[9:11]
        axis_status = str(bin(int(axis_status,16)))[2:].rjust(8,'0') # hexstring to binary
        servo_is_stop = axis_status[7]
        is_home = axis_status[5:7]
        servo_is_on = axis_status[4]
        command_is_done = axis_status[3]

        return command_is_done


    def checksum(self, string_command):
        checksum = 0
        for i in range(0, len(string_command)):
            checksum = checksum + ord(string_command[i])
        checksum = hex(int(checksum)).lstrip('0x')
        checksum = checksum[-2:]
        return checksum


if __name__ == "__main__":
    # check the port at Device Manager for Window
    # set to /dev/ttyUSB0 for linux
    set_port = 'COM5'
    set_baudrate = 38400
    set_timeout = 3

    # initialize
    robot = IAI(set_port, set_baudrate, set_timeout)
    robot.set_workspace((20, 150), 130, 130)

    # while True:
    #     robot.check_status('xyz')

    # move in absolute coordinate
    # positions = [(10, 10, 0), (20, 10, 0), (30, 10, 0)]
    # for position in positions:
    #     robot.move('absolute', 'xyz', 0.3, 100, position[0], position[1], 0)
    #     time.sleep(2)
    #     robot.drill()

    # move in workspace coordinate
    positions = [(10, 10), (20, 10), (30, 10)]
    for position in positions:
        robot.move_in_workspace(position[0], position[1])
        while(robot.check_status() != '1'):
            time.sleep(0.1)
        robot.drill()
