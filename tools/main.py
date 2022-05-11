import serial
import sys
from time import sleep

READ_INPUT = 0x0101
SET_OUTPUT = 0x0202
SET_MULTI_OUTPUT = 0x0203
SET_ADDRESS = 0xFF03

MINION_ADDRESS = 0x14030108


def crc(data):
    res = 0
    for d in data:
        res += d
    res = res & 0xFF
    return res

def build_packet(command, dest, data=[]):
    packet = [2, 1, 0, 0, (dest >> 24) & 0xFF,(dest >> 16) & 0xFF,(dest >> 8) & 0xFF,dest &0xFF,0,0,0,0,(command >> 8) & 0xFF,command & 0xFF] + data
    packet[2] = len(packet) + 1
    packet = packet + [crc(packet)]
    return packet

def read_inputs(port):
    port.write(build_packet(READ_INPUT, MINION_ADDRESS))
    res = port.read(15)
    print(f"0x{int(res[13]):02X}")

def set_output(port, output):
    port.write(build_packet(SET_OUTPUT, MINION_ADDRESS, [output, 0, 1]))
    print([f"0x{int(x):02X}" for x in port.read(15)])


def set_multi_output(port, output):
    port.write(build_packet(SET_MULTI_OUTPUT, MINION_ADDRESS, [output, 2, 0, 1, 0, 1]))
    print([f"0x{int(x):02X}" for x in port.read(15)])


def main():
    port = serial.Serial(sys.argv[1], baudrate=9600, timeout=0.2)

    port.write(build_packet(SET_ADDRESS, 1, [(MINION_ADDRESS >> 24) & 0xFF, (MINION_ADDRESS >> 16) & 0xFF, (MINION_ADDRESS >> 8) & 0xFF, MINION_ADDRESS & 0xFF]))
    res = port.read(255)
    print(res)

    #set_multi_output(port, 1)

    read_inputs(port)
    for i in range(4):
        sleep(0.3)
        set_output(port, 1 << i)



if __name__ == "__main__":
    main()