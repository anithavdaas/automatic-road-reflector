import time
import RPi.GPIO as GPIO  # If using Raspberry Pi
# Or: import machine # If using MicroPython (e.g., ESP32/ESP8266)

# Configuration (adjust these values as needed)
LED_PIN = 17  # GPIO pin connected to the LED/reflector
LDR_PIN = 4   # GPIO pin connected to the Light Dependent Resistor (LDR)
THRESHOLD = 500 # Threshold value for darkness detection (adjust based on LDR readings)
DELAY_ON = 5    # Time LED stays on (seconds)
DELAY_OFF = 1   # Time LED stays off (seconds)

# Setup (Raspberry Pi example)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(LDR_PIN, GPIO.IN)

def read_ldr():
    # Example using a simple digital approximation (for basic LDR sensing)
    # For more accurate analog readings, use an ADC (Analog-to-Digital Converter)
    count = 0
    GPIO.setup(LDR_PIN, GPIO.OUT)
    GPIO.output(LDR_PIN, GPIO.LOW)
    time.sleep(0.1)

    GPIO.setup(LDR_PIN, GPIO.IN)
    while (GPIO.input(LDR_PIN) == GPIO.LOW):
        count += 1
    return count

def control_led(state):
    GPIO.output(LED_PIN, state)

try:
    while True:
        ldr_value = read_ldr()
        print(f"LDR Value: {ldr_value}") #for debug purposes

        if ldr_value < THRESHOLD: # Darkness detected
            control_led(GPIO.HIGH) # Turn LED on
            time.sleep(DELAY_ON)
            control_led(GPIO.LOW)  # Turn LED off
            time.sleep(DELAY_OFF)
        else:
            control_led(GPIO.LOW)  # Keep LED off in daylight
            time.sleep(1) # Check again every second.

except KeyboardInterrupt:
    print("Program stopped by user")
    GPIO.cleanup() # Clean up GPIO settings

# MicroPython example (ESP32/ESP8266)
"""
from machine import Pin, ADC
import time

LED_PIN = 2
LDR_PIN = 34 #Example ADC pin

THRESHOLD = 500
DELAY_ON = 5
DELAY_OFF = 1

led = Pin(LED_PIN, Pin.OUT)
ldr = ADC(Pin(LDR_PIN))
ldr.atten(ADC.ATTN_11DB) # Adjust attenuation as needed

def read_ldr():
    return ldr.read()

def control_led(state):
    led.value(state)

try:
    while True:
        ldr_value = read_ldr()
        print(f"LDR Value: {ldr_value}")

        if ldr_value < THRESHOLD:
            control_led(1)
            time.sleep(DELAY_ON)
            control_led(0)
            time.sleep(DELAY_OFF)
        else:
            control_led(0)
            time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by user")

"""# automatic-road-reflector
