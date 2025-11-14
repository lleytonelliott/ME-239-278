from machine import Pin, ADC, PWM, I2C
import pinout
import time

pressure_1_adc = ADC(Pin(pinout.PRESSURE_PIN_1))
pressure_1_adc.atten(ADC.ATTN_11DB) # Set for 0-3.3V range
raw_val = pressure_1_adc.read_u16()

while True:
    # Read the 16-bit raw value (0-65535)
    raw_val = pressure_1_adc.read_u16()
    voltage = (raw_val / 65535) * 3.3
    print(f"Raw: {raw_val:<5}  |  Voltage: {voltage:.2f}V")
    time.sleep(0.5)