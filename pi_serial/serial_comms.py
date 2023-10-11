import serial
import pynput
import keybinds

# Defines for our message types
MSG_TYPE_BTN = 1
MSG_TYPE_ADC = 2
MSG_TYPE_NFC = 3

ADC1_IDX = 0
ADC2_IDX = 1

# Value defines for our button types
MSG_SELECT_USER1    = 1
MSG_SELECT_USER2    = 2
MSG_SELECT_USER3    = 3
MSG_SELECT_USER4    = 4
MSG_VIEW_UP         = 5
MSG_VIEW_DOWN       = 6
MSG_VIEW_LEFT       = 7
MSG_VIEW_RIGHT      = 8

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
    else :
        print("I dont know what message this is? I just got: ", msg_type)


def handle_button (btn_value : int) :
    # Depending on button, change the operation
    if btn_value == MSG_SELECT_USER1 :
        print("MESSAGE_BUTTON_USER: User 1 selected")
        press_key(keybinds.SELECT_USER1)
    elif btn_value == MSG_SELECT_USER2 :
        print("MESSAGE_BUTTON_USER: User 2 selected")
        press_key(keybinds.SELECT_USER2)
    elif btn_value == MSG_SELECT_USER3 :
        print("MESSAGE_BUTTON_USER: User 3 selected")
        press_key(keybinds.SELECT_USER3)
    elif btn_value == MSG_SELECT_USER4 :
        print("MESSAGE_BUTTON_USER: User 4 selected")
        press_key(keybinds.SELECT_USER4)
    elif btn_value == MSG_VIEW_UP :
        print("MESSAGE_BUTTON_SCRN: View up")
        press_key(keybinds.VIEW_SCROLL_UP)
    elif btn_value == MSG_VIEW_DOWN :
        print("MESSAGE_BUTTON_SCRN: View down")
        press_key(keybinds.VIEW_SCROLL_DOWN)
    elif btn_value == MSG_VIEW_LEFT :
        print("MESSAGE_BUTTON_SCRN: View left")
        press_key(keybinds.VIEW_WEEK_BACK)
    elif btn_value == MSG_VIEW_RIGHT :
        print("MESSAGE_BUTTON_SCRN: View right")
        press_key(keybinds.VIEW_WEEK_FORWARD)
    else :
        print("Unknown button value received...")


def handle_adc (adc_index : int, adc_value : int) :

    global adc1_lastthresh, adc2_lastthresh

    if adc_index == ADC1_IDX : #ADC1 time! Controls screen
        if adc1_lastthresh < adc_value : # New value is higher
            print("MESSAGE_ADC_SCRN: Screen brightness up")
            press_key(keybinds.SCREEN_BRIGHT_UP)
        else : # New value is lower
            print("MESSAGE_ADC_SCRN: Screen brightness down")
            press_key(keybinds.SCREEN_BRIGHT_DOWN)
        adc1_lastthresh = adc_value

    else : # ADC2 time! Controls 
        if adc2_lastthresh < adc_value : # New value is higher
            print("MESSAGE_ADC_USER: Time to turn brightness up!")
            press_key(keybinds.USERS_BRIGHT_UP)
        else : # New value is lower
            print("MESSAGE_ADC_USER: Time to turn brightness DOWN!")
            press_key(keybinds.USERS_BRIGHT_DOWN)
        adc2_lastthresh = adc_value

# Need to change COM5 to 
def main() :
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
    data = ser.readline()

    try :
        while True :
            if data :
                print(data)
                parse_bytes(data)
            data = ser.readline()
    except KeyboardInterrupt :
        exit(1)

if __name__ == "__main__" :
    main()
