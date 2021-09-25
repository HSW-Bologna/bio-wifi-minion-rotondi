import serial

def crc(data):
    res = 0
    for d in data:
        res += d
    res = res & 0xFF
    return res

def main():
    port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=0.1)

    packet = [2, 1, 16, 0, 1,2,3,4,0,0,0,0,0,1,2]
    port.write(packet + [crc(packet)])
    print([f"0x{int(x):02X}" for x in port.read(256)])


if __name__ == "__main__":
    main()