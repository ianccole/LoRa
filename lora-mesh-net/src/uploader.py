import argparse


def main():
    parser = argparse.ArgumentParser(description='Description of your program')
    parser.add_argument('-f','--file',  help='firmware hex file', required=True)
    parser.add_argument('-p','--port',  help='serial port', required=True)
    parser.add_argument('-b','--baud',  help='baud rate', required=True)
    args = vars(parser.parse_args())

if __name__ == '__main__':
    main()