import serial
import time

cnt=0

try:
    print('opening port...')
    ser=serial.Serial(port = "/dev/ttyUSB0", baudrate = 9600, timeout= 0.5)
except Exception as e:
    print(e)
finally:
    print('port open, writing')
    try:
        while True:
            ser.write((f"hello {cnt}\r\n").encode('ascii'))
            cnt=cnt+1
            time.sleep(2)
            if ser.in_waiting>0:
                data=ser.read_all()
                print(f"recv: {data.decode('ascii')}")
    except KeyboardInterrupt:
        ser.close()
        print('\nport closed')