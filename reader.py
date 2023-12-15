import serial
import datetime

def convert_time(time):
    return str(datetime.datetime.fromtimestamp(int(time)/1000000) + datetime.timedelta(365*(2023-1970)+13, hours=18))

def convert_value(value):
    return value.decode("utf-8") + "mv"


def convert_trigger(trigger):
    if trigger == "0":
        return "no trigger"
    elif trigger == "1":
        return "up trigger"
    elif trigger == "2":
        return "down trigger"
    else:
        return "invalid trigger"


def main():
    s = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1)
    s.write(b"hello")
    count = 0
    while True:
        line = s.readline()
        if line == b'':
            break

        line = line.split(b"[")[1].split(b"]")[0]
        line = line.split(b",")

        print(convert_time(line[0]), convert_value(line[1]), convert_trigger(line[2]))
    print(count)

if __name__ == "__main__":
    main()