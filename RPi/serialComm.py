import serial

"""This code establish a communication between PC and ARM mc using the USART port """

"""
** the output like this :

    enter somthing:
    w
    Welcome!

    enter somthing:
    g
    Goodbye!
    .
    .
"""

PORT = '/dev/ttyUSB0'
BAUD_RATE = 57600

ser = serial.Serial(PORT, BAUD_RATE)

while True:
    val = input("enter something:\n")
    ser.write(val.encode())
    output=ser.readlines(1)
    print(output[0].decode())