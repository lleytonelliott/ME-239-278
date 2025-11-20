# hardware.py
# Low-Level Hardware Helper Functions for ESP32
# Requires 'pinout.py' to be on the device.

from machine import Pin, ADC, PWM, I2C
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import board
import pinout

# --- Global Hardware Objects ---
# These will be initialized by initialize_hardware()
motor_pwm = None
motor_dir = None
motor_en = None
motor_current_adc = None
knee_pot_adc = None
i2c = None
reset_button = None

# --- Sensor Characteristics (EXAMPLE VALUES) ---
# You MUST update these based on your sensors.
# For current sensor (e.g., ACS712-05B):
# 2.5V = 0 Amps, Sensitivity = 185mV/A
CURRENT_V_OFFSET = 2.5  # Volts
CURRENT_SENSITIVITY = 0.185 # Volts per Amp
# For knee potentiometer:
# Map the 0-65535 raw value to your angle range
POT_MIN_RAW = 0
POT_MAX_RAW = 65535
POT_MIN_ANGLE = 0.0   # Degrees
POT_MAX_ANGLE = 180.0 # Degrees


def initialize_hardware():

    global motor_pwm_forward, motor_pwm_reverse, motor_en, motor_current_adc, knee_pot_adc, i2c, reset_button
    
    # --- Initialize Motor Pins ---
    motor_pwm_forward = PWM(Pin(pinout.MOTOR_PWM_PIN_FORWARD), freq=20000, duty_u16=0)
    motor_pwm_reverse = PWM(Pin(pinout.MOTOR_PWM_PIN_REVERSE), freq=20000, duty_u16=0)

    # --- Initialize Analog Sensors ---
    pressure_pin_1 = ADC(Pin(pinout.PRESSURE_PIN_1))
    pressure_pin_1.atten(ADC.ATTN_11DB) # Set for 0-3.3V range
    pressure_pin_2 = ADC(Pin(pinout.PRESSURE_PIN_2))
    pressure_pin_2.atten(ADC.ATTN_11DB) # Set for 0-3.3V range
    pressure_pin_3 = ADC(Pin(pinout.PRESSURE_PIN_3))
    pressure_pin_3.atten(ADC.ATTN_11DB) # Set for 0-3.3V range
    pressure_pin_4 = ADC(Pin(pinout.PRESSURE_PIN_4))
    pressure_pin_4.atten(ADC.ATTN_11DB) # Set for 0-3.3V range

    # --- Initialize I2C Sensors ---
    i2c = I2C(0, sda=Pin(pinout.IMU_SDA_PIN), scl=Pin(pinout.IMU_SCL_PIN), freq=400000)

    # --- Initialize UI Pins ---
    reset_button = Pin(pinout.RESET_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    
    print("Hardware Initialized.")
    print("I2C devices found at:", [hex(addr) for addr in i2c.scan()])


def drive_motor(signal):
    """
    Drives the motor.
    :param signal: A float from -1.0 (full reverse) to 1.0 (full forward).
    """
    if motor_pwm_forward is None or motor_pwm_reverse is None:
        print("Hardware not initialized!")
        return
    
    # Set direction
    if signal >= 0:
        duty_cycle = int(abs(signal) * 255)
        motor_pwm_forward.duty(duty_cycle)
        motor_pwm_reverse.duty(0)
    else:
        duty_cycle = int(abs(signal) * 255)
        motor_pwm_forward.duty(0)
        motor_pwm_reverse.duty(duty_cycle)
    

def read_motor_current():
    """
    Reads the motor current from the ADC sensor.
    Returns current in Amps.
    """
    if motor_current_adc is None:
        print("Hardware not initialized!")
        return 0.0

    # Read raw 16-bit value (0-65535)
    raw_val = motor_current_adc.read_u16()
    
    # Convert to voltage (0.0 - 3.3)
    voltage = (raw_val / 65535) * 3.3
    
    # Convert voltage to current based on sensor specs
    current = (voltage - CURRENT_V_OFFSET) / CURRENT_SENSITIVITY
    
    return current


def read_knee_angle():
    """
    Reads the knee angle from the potentiometer.
    Returns angle in degrees.
    """
    if knee_pot_adc is None:
        print("Hardware not initialized!")
        return 0.0
        
    raw_val = knee_pot_adc.read_u16()
    
    # Map raw value to angle (linear interpolation)
    # This maps [POT_MIN_RAW, POT_MAX_RAW] to [POT_MIN_ANGLE, POT_MAX_ANGLE]
    angle = (raw_val - POT_MIN_RAW) * (POT_MAX_ANGLE - POT_MIN_ANGLE) / (POT_MAX_RAW - POT_MIN_RAW) + POT_MIN_ANGLE
    
    return angle


def read_imu_data():
    """
    Reads data from the IMU.
    *** THIS IS A STUB - REQUIRES A DRIVER FOR YOUR SPECIFIC IMU ***
    
    You must find a MicroPython driver (e.g., mpu6050.py) for your
    IMU, add it to your ESP32, and import it here.
    
    Returns: (z_orientation, z_angular_velocity) - EXAMPLE TUPLE
    """
    if i2c is None:
        print("Hardware not initialized!")
        return (0.0, 0.0)
    
    # --- EXAMPLE of what driver code would look like ---
    # try:
    #     # This is a FAKE function. Your driver will have a real one.
    #     # accel_data = imu_driver.read_accel()
    #     # gyro_data = imu_driver.read_gyro()
    #
    #     # Process data to get z_orientation and z_angular_velocity
    #     z_ori = 0.0 # Calculated from accel/gyro
    #     z_ori_dot = 0.0 # Gyro z-axis
    #     return (z_ori, z_ori_dot)
    #
    # except OSError as e:
    #     print(f"I2C Error: {e}")
    #     return (0.0, 0.0) # Return safe values
    
    # Placeholder return:
    return (0.0, 0.0)


def read_seat_sensor():
    """
    Reads data from the seat force sensor.
    *** THIS IS A STUB ***
    
    This could be another ADC pin, I2C, or SPI.
    Returns: Force in Newtons (or some proportional value)
    """
    # If it's another ADC:
    # raw_val = seat_adc.read_u16()
    # force = map_adc_to_force(raw_val)
    # return force
    
    return 0.0 # Placeholder


def reset_button_pressed():
    """
    Checks if the reset button is currently pressed.
    Returns True if pressed, False otherwise.
    """
    if reset_button is None:
        print("Hardware not initialized!")
        return False
        
    # Reads the pin value.
    # We used PULL_UP, so pin is HIGH (1) when not pressed
    # and LOW (0) when pressed.
    return reset_button.value() == 0