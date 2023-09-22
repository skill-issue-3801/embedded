import serial
import pynput

# Need to change COM5 to 
def main() :
    keyboard = pynput.keyboard.Controller()

    ser = serial.Serial('COM5', 115200, timeout=0.1)
    data = ser.read(9)

    try :
        while True :
            if data :
                keyboard.press(chr(data[0]))
                keyboard.release(chr(data[0]))
            data = ser.read(9)
    except KeyboardInterrupt :
        exit(1)

    


if __name__ == "__main__" :
    main()