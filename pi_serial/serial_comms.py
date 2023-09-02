import serial

# Need to change COM5 to 
def main() :
    ser = serial.Serial('COM5', 115200, timeout=1)
    data = ser.read(9)

    try :
        while True :
            print(data)
            data = ser.read(9)
    except KeyboardInterrupt :
        exit(1)

    


if __name__ == "__main__" :
    main()