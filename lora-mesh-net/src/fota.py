import argparse
import serial
import pexpect.fdpexpect
# from pexpect_serial import SerialSpawn

import time

def loadHex(args):
    seq = 0
    # ser = serial.Serial(args.port, args.baud)

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.setDTR(False)
    ser.open()

    client = pexpect.fdpexpect.fdspawn(ser)
    client.timeout = 5

    client.expect('RF95 ready\(mem.*\)')
    print client.before.strip()
    print client.after.strip()

    # with serial.Serial(args.port, args.baud) as ser:
    #     ss = SerialSpawn(ser)
    #     ss.expect('ready')

    with open(args.file, 'r') as f:
        for line in f:
            fota = 'F %s %s %s\n' %(args.node, seq, line.strip())
            client.write(fota)
            ii = client.expect(['ACK', 'NAK'])
            print(client.before.strip(),client.after.strip())
            if ii == 1:
                break
            seq += 1


def main():
    parser = argparse.ArgumentParser(description='Firmware OTA uploader')
    parser.add_argument('-f','--file',  help='firmware hex file', default='.pio/build/moteino8mhz/firmware.hex')
    parser.add_argument('-p','--port',  help='serial port', default='/dev/ttyUSB0')
    parser.add_argument('-b','--baud',  help='baud rate', default='57600')
    parser.add_argument('-n','--node',  help='node', default='90')
    args = parser.parse_args()

    loadHex(args)

if __name__ == '__main__':
    main()