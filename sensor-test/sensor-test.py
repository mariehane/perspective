import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

try:
    while True:
        data = arduino.readline()
        if data:
            print(data)
            strings = data[:-2].split(b",")
            print(strings)
            if len(strings) == 2:
                str1, str2 = strings
                val1 = int(str1)
                val2 = int(str2)
                print("vals:", val1, val2)

except Exception as e:
    print(e)
    arduino.close()
