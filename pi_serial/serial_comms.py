import serial
import pynput
import keybinds

# Defines for our message types
MSG_TYPE_BTN = 1
MSG_TYPE_ADC = 2
MSG_TYPE_NFC = 3

ADC1_IDX = 0
ADC2_IDX = 1

# Set at 5, assume thats our default max brightness
adc1_lastthresh = 5
adc2_lastthresh = 5

# Keyboard interation object
keyboard = pynput.keyboard.Controller()


def press_key(key_character: chr) :
    keyboard.press(key_character)
    keyboard.release(key_character)


def parse_bytes(data : bytes) :
    msg_type = data[0]

    if msg_type is MSG_TYPE_BTN :
        handle_button(data[1])
    elif msg_type is MSG_TYPE_ADC :
        handle_adc(data[1], data[2])
    elif  msg_type is MSG_TYPE_NFC :
        print("NFC time")
    else :
        print("I dont know what message this is?")


def handle_button (btn_index : int) :
    # Depending on button, change the operation
    print("Just got a message from button ", btn_index)


def handle_adc (adc_index : int, adc_value : int) :

    global adc1_lastthresh, adc2_lastthresh

    if adc_index == ADC1_IDX : #ADC1 time!
        if adc1_lastthresh < adc_value : # New value is higher
            print("ADC1: Time to turn brightness up!")
        else : # New value is lower
            print("ADC1: Time to turn brightness DOWN!")
        adc1_lastthresh = adc_value

    else : # ADC2 time!
        if adc2_lastthresh < adc_value : # New value is higher
            print("ADC2: Time to turn brightness up!")
        else : # New value is lower
            print("ADC2: Time to turn brightness DOWN!")
        adc2_lastthresh = adc_value

# Need to change COM5 to 
def main() :
    ser = serial.Serial('COM5', 115200, timeout=0.1)
    data = ser.readline()

    try :
        while True :
            if data :
                print(data)
                
                parse_bytes(data)
            data = ser.read(9)
    except KeyboardInterrupt :
        exit(1)

    


if __name__ == "__main__" :
    main()