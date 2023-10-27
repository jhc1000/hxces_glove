import serial
import time

#시리얼포트 객체 ser을 생성
#pc와 스위치 시리얼포트 접속정보
ser = serial.Serial(
    port = 'COM14', 
    baudrate=115200, 
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=0
    )

#시리얼포트 접속
ser.isOpen()

#시리얼포트 번호 출력
print(ser.name)

try:
    i = 0
    while(1):
        msg = 'A{a}B{b}C{c}D{d}E{e}\n'.format(a=i, b=i, c=i, d=i, e=i)
        print(msg)
        ser.write(msg.encode('utf-8'))
        time.sleep(0.1)
        i = (i + 1) % 90

except KeyboardInterrupt:
    print(KeyboardInterrupt)

finally:
    print('end haptic node')
