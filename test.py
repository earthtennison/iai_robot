print(hex(0b11))
print(hex(11))


def check_status(axis):
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

    # print(axis_pattern)
    byte_format = 2
    axis_pattern = hex(axis_pattern).lstrip('0x').rjust(byte_format,'0')


    string_command = '!00' + message_id + axis_pattern
    cs = checksum(string_command)
    string_command += cs
    # print(cs)
    # print(string_command)
    # ser.write((string_command + '\r\n').encode())
    # read = ser.readline().decode()
    # print(read + '\n')

def checksum(string_command):
    checksum = 0
    for i in range(0, len(string_command)):
        checksum = checksum + ord(string_command[i])
    checksum = hex(int(checksum)).lstrip('0x')
    checksum = checksum[-2:]
    return checksum
def decimalToBinary(n):
    return "{0:b}".format(int(n)).rjust(8, '0')

def hex_to_binary( hex_code ):
  bin_code = bin( hex_code )[2:]
  padding = (4-len(bin_code)%4)%4
  return '0'*padding + bin_code

# check_status("xyz")
# print(int("1C",16))
# print(decimalToBinary(28))

# print(bin(0xA))
print(str(bin(int("1C",16)))[2:].rjust(8,'0'))