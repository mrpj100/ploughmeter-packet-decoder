# serial-based decoder for Cryoegg packets received from
# Radiocrafts Wireless M-Bus module in master mode


# configuration

import serial
import math
import datetime



def separate_packet (raw_data):
    isinstance(raw_data, bytes) or exit (99)

    packet_dict = {}

    # c field
    # 2 byte M ID
    # 4 byte U ID
    # 1 byte ver
    # 1 byte dev
    # CI field
    # user data
    # RSSI


    length = len(raw_data)
    last_user_byte = length - 2
    rssi_byte = length - 1

    packet_dict['C-Field'] = raw_data[0]
    packet_dict['M-Field'] = int.from_bytes(raw_data[1:3], byteorder = 'little', signed=False)
    packet_dict['U-Field'] = int.from_bytes(raw_data[3:7], byteorder = 'little', signed=False)
    packet_dict['Version'] = raw_data[7]
    packet_dict['DeviceID'] = raw_data[8]
    packet_dict['CI-Field'] = raw_data[9]
    ######################## USED TO BE last_user_byte+1, Hashem changed it to + 2 because it was cutting off data
    packet_dict['UserData'] = raw_data[10:last_user_byte+2] # store user data as bytes
    packet_dict['RSSI'] = -1 * (raw_data[rssi_byte]) /2  # convert reported RSSI value to dBm

    return packet_dict

def decode_cryoegg_data (cryoegg_data):

    data_dict = {}

    data_dict['Raw-Conductivity'] = int.from_bytes(cryoegg_data[0:2], byteorder = 'little', signed=False)
    data_dict['Raw-TempPT1000'] = int.from_bytes(cryoegg_data[2:4], byteorder = 'little', signed=False)
    data_dict['Raw-Pressure'] = int.from_bytes(cryoegg_data[4:6], byteorder = 'little', signed=False)
    data_dict['Raw-TempKeller'] = int.from_bytes(cryoegg_data[6:8], byteorder = 'little', signed=False)

    return data_dict
def decode_cryowurst_data (cryowurst_data):

    data_dict = {}

    data_dict['Raw-Temperature'] = int.from_bytes(cryowurst_data[0:2], byteorder = 'big', signed=True)

    data_dict['Raw-IMU_MAG_X'] = int.from_bytes(cryowurst_data[2:4], byteorder = 'big', signed=True)
    data_dict['Raw-IMU_MAG_Y'] = int.from_bytes(cryowurst_data[4:6], byteorder = 'big', signed=True)
    data_dict['Raw-IMU_MAG_Z'] = int.from_bytes(cryowurst_data[6:8], byteorder = 'big', signed=True)

    data_dict['Raw-TILT_ACC_X'] = int.from_bytes(cryowurst_data[8:10], byteorder = 'big', signed=True)
    data_dict['Raw-TILT_ACC_Y'] = int.from_bytes(cryowurst_data[10:12], byteorder = 'big', signed=True)
    data_dict['Raw-TILT_ACC_Z'] = int.from_bytes(cryowurst_data[12:14], byteorder = 'big', signed=True)

    data_dict['Raw-TILT_PITCH_X'] = int.from_bytes(cryowurst_data[14:16], byteorder = 'big', signed=True)
    data_dict['Raw-TILT_ROLL_Y'] = int.from_bytes(cryowurst_data[16:18], byteorder = 'big', signed=True)

    data_dict['Raw-Conductivity'] = int.from_bytes(cryowurst_data[18:20], byteorder = 'big', signed=True)

    data_dict['Raw-Pressure'] = int.from_bytes(cryowurst_data[20:22], byteorder = 'big', signed=True)
    #data_dict['Raw-TILT_ACC_Y'] = int.from_bytes(cryowurst_data[22:24], byteorder = 'big', signed=True)
    #data_dict['Raw-TILT_ACC_Z'] = int.from_bytes(cryowurst_data[24:26], byteorder = 'big', signed=True)

    #data_dict['Raw-TILT_PITCH_X'] = int.from_bytes(cryowurst_data[26:28], byteorder = 'big', signed=True)
    #data_dict['Raw-TILT_ROLL_Y'] = int.from_bytes(cryowurst_data[28:30], byteorder = 'big', signed=True)

    return data_dict

def decode_ploughmeter_data(ploughmeter_data):
    
    data_dict = {}

    data_dict['Raw-Temperature'] = int.from_bytes(ploughmeter_data[0:2], byteorder = 'big', signed=True)

    data_dict['Raw-IMU_MAG_X'] = int.from_bytes(ploughmeter_data[2:4], byteorder = 'big', signed=True)
    data_dict['Raw-IMU_MAG_Y'] = int.from_bytes(ploughmeter_data[4:6], byteorder = 'big', signed=True)
    data_dict['Raw-IMU_MAG_Z'] = int.from_bytes(ploughmeter_data[6:8], byteorder = 'big', signed=True)

    data_dict['Raw-TILT_ACC_X'] = int.from_bytes(ploughmeter_data[8:10], byteorder = 'big', signed=True)
    data_dict['Raw-TILT_ACC_Y'] = int.from_bytes(ploughmeter_data[10:12], byteorder = 'big', signed=True)
    data_dict['Raw-TILT_ACC_Z'] = int.from_bytes(ploughmeter_data[12:14], byteorder = 'big', signed=True)

    data_dict['Raw-TILT_PITCH_X'] = int.from_bytes(ploughmeter_data[14:16], byteorder = 'big', signed=True)
    data_dict['Raw-TILT_ROLL_Y'] = int.from_bytes(ploughmeter_data[16:18], byteorder = 'big', signed=True)

    data_dict['Raw-Conductivity'] = int.from_bytes(ploughmeter_data[18:20], byteorder = 'big', signed=True)

    data_dict['Raw-Pressure'] = int.from_bytes(ploughmeter_data[20:22], byteorder = 'big', signed=True)
    #data_dict['Raw-TILT_ACC_Y'] = int.from_bytes(ploughmeter_data[22:24], byteorder = 'big', signed=True)
    #data_dict['Raw-TILT_ACC_Z'] = int.from_bytes(ploughmeter_data[24:26], byteorder = 'big', signed=True)

    #data_dict['Raw-TILT_PITCH_X'] = int.from_bytes(ploughmeter_data[26:28], byteorder = 'big', signed=True)
    #data_dict['Raw-TILT_ROLL_Y'] = int.from_bytes(ploughmeter_data[28:30], byteorder = 'big', signed=True)
    data_dict['Raw-LoadCell-Ch1'] = int.from_bytes(ploughmeter_data[22:25], byteorder= 'big', signed=True)
    data_dict['Raw-LoadCell-Ch2'] = int.from_bytes(ploughmeter_data[25:28], byteorder= 'big', signed=True)

    return data_dict

def convert_keller_pressure (raw_pressure):
    # converts Keller digital values to real pressures in bar
    bar_pressure = 0.0

    keller_max_bar = 250 # 100 bar for demo kit sensor - 250 bar for real sensor
    keller_min_bar = 0 # 0 bar for our sensor - it doesn't go down to vacuum

    pressure_range = keller_max_bar - keller_min_bar

    bar_pressure = ((raw_pressure - 16384) * (pressure_range / 32768)) + keller_min_bar

    return bar_pressure

def convert_keller_temperature (raw_temperature):
    # converts Keller digital values to real temperatures in Celcius
    # reduces precision to 12-bits as recommended by Keller

    celcius_temp = 0.0

    celcius_temp =  (((raw_temperature >> 4) - 24) * 0.05) - 50

    return celcius_temp


# Temperature of RTD Function                             T_rtd
#/ input: r = resistance of RTD
# output: T_rtd() = corresponding temperature of RTD
# Calculates temperature of RTD as a function of resistance via
# a direct mathematical method.
# Modified and ported to Python from Analog Devices sample code - see App Note AN-0970

def Temperature_rtd (r):
    Z1 = (-3.9083E-3)    # Z1 coef. of positive reverse transfer function
    Z2 = (17.58480889E-6) # Z2 coef. of positive reverse transfer function
    Z3 = (-23.10E-9)     # Z3 coef. of positive reverse transfer function
    Z4 = (-1.155E-6)     # Z4 coef. of positive reverse transfer function
    TMIN = (-200)  # minimum temperature [degC]
    TMAX = (850)     # maximum temperature [degC]
    RMIN = 18.52008 # minimum resistance [ohms]
    RMAX = 390.4811  # maximum resistance [ohms]

    # first determine if input resistance is within spec'd range
    if (r<RMIN):           # if input is under-range..
        t = TMIN           # ..then set to minimum of range
    elif (r>RMAX):      # if input is over-range..
        t = TMAX          # ..then set to maximum of range

    # if input (r) is within range, then solve for output.
    else:

        # if r < threshold, use negative transfer function
        if (r<95.1):
            t=-242.0906+2.227625*r+2.517790E-3*pow(r,2)-5.861951E-6*pow(r,3)

        # if r >= threshold, use positive transfer function
        else:
             t=(Z1+math.sqrt(Z2+Z3*r))/Z4


    return t


def ConvertADCValueToTemperature(raw_result):

	# PT1000 interface circuit values
    excitation_current = 100e-6
    reference_resistance = 820
    amplifier_gain = 116.11
    supply_voltage = 3.3

    reference_voltage = excitation_current * reference_resistance

    ADC_max = 4095

    pt100_equiv_resistance = 1000 * (((supply_voltage * raw_result) / (ADC_max * amplifier_gain)) + reference_voltage)

    temperature = Temperature_rtd(pt100_equiv_resistance)

    return temperature


# MAIN FUNCTION

# initialisation

rx_port = serial.Serial('COM11', baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=None)

log_file_name = datetime.datetime.strftime(datetime.datetime.utcnow(), '%Y-%m-%d-%H-%M-%S') + "ploughmeterlog.csv"

log_file = open(log_file_name, "w")

log_file.write("Timestamp, UID, RSSI, RawTemperature, RawIMU_M_X, RawIMU_M_Y, RawIMU_M_Z, RawTIL_A_X, RawTIL_A_Y, RawTIL_A_Z, RawTIL_T_X, RawTIL_T_Y, RawCOND, RawPRESSURE, LoadCellCh1, LoadCellCh2")
log_file.write("\n")
print("Timestamp,              UID       RSSI       Temp         IMU_M_X    IMU_M_Y    IMU_M_Z   TIL_A_X   TIL_A_Y   TIL_A_Z   TIL_T_X  TIL_T_Y  COND PRESSURE     LoadCellCh1 LoadCellCh2")

try:
    while True:

        #print("getting packet")
        # wait for a new packet - first byte is length
        packet_length = int.from_bytes(rx_port.read(size = 1), byteorder = 'little', signed=False)

        packet_rx_time = datetime.datetime.utcnow()

        #print(repr(packet_rx_time))

        #print(packet_rx_time.isoformat(timespec='milliseconds'))

        #print("reading packet")
        # read the rest of the packet
        packet_bytes = rx_port.read(size = packet_length)

        # separate the packet into header, user data and trailer
        #print("packet bytes: "+repr(packet_bytes))

        #print("seperate packet")
        separated_packet = separate_packet(packet_bytes)

        #print(repr(separated_packet))

        #print("user data = "+repr(len(separated_packet['UserData']))+" bytes")

        ##cryoegg_data = decode_cryoegg_data(separated_packet['UserData'])

        #print("decode packet")
        ploughmeter_data = decode_ploughmeter_data(separated_packet['UserData'])

        #print(ploughmeter_data)

        #print("------------------------")

        TMP117_temperature_decode = ploughmeter_data['Raw-Temperature'] * 0.0078125
        #print("TMP117 Temperature: " + str(TMP117_temperature_decode) + " C")

        #IMU_accelX_decode = ploughmeter_data['Raw-IMU_ACC_X'] * 0.06103515625
        #print("IMU Accel X: " + str(IMU_accelX_decode) + " mg")
        #IMU_accelY_decode = ploughmeter_data['Raw-IMU_ACC_Y'] * 0.06103515625
        #print("IMU Accel Y: " + str(IMU_accelY_decode) + " mg")
        #IMU_accelZ_decode = ploughmeter_data['Raw-IMU_ACC_Z'] * 0.06103515625
        #print("IMU Accel Z: " + str(IMU_accelZ_decode) + " mg")

        #IMU_gyroX_decode = ploughmeter_data['Raw-IMU_GYR_X'] * 0.0076335877862595
        #print("IMU Gyro X: " + str(IMU_gyroX_decode) + " °/sec")
        #IMU_gyroY_decode = ploughmeter_data['Raw-IMU_GYR_Y'] * 0.0076335877862595
        #print("IMU Gyro Y: " + str(IMU_gyroY_decode) + " °/sec")
        #IMU_gyroZ_decode = ploughmeter_data['Raw-IMU_GYR_Z'] * 0.0076335877862595
        #print("IMU Gyro Z: " + str(IMU_gyroZ_decode) + " °/sec")

        IMU_magX_decode = ploughmeter_data['Raw-IMU_MAG_X'] * 0.15
        #print("IMU Mag X: " + str(IMU_magX_decode) + " μT")
        IMU_magY_decode = ploughmeter_data['Raw-IMU_MAG_Y'] * 0.15
        #print("IMU Mag Y: " + str(IMU_magY_decode) + " μT")
        IMU_magZ_decode = ploughmeter_data['Raw-IMU_MAG_Z'] * 0.15
        #print("IMU Mag Z: " + str(IMU_magZ_decode) + " μT")

        TILT_accelX_decode = ploughmeter_data['Raw-TILT_ACC_X']
        #print("TILT Accel X: " + str(TILT_accelX_decode) + " mg")
        TILT_accelY_decode = ploughmeter_data['Raw-TILT_ACC_Y']
        #print("TILT Accel Y: " + str(TILT_accelY_decode) + " mg")
        TILT_accelZ_decode = ploughmeter_data['Raw-TILT_ACC_Z']
        #print("TILT Accel Z: " + str(TILT_accelZ_decode) + " mg")

        TILT_pitchX_decode = ploughmeter_data['Raw-TILT_PITCH_X'] * 0.1
        #print("TILT Pitch X: " + str(TILT_pitchX_decode) + " °")
        TILT_rollY_decode = ploughmeter_data['Raw-TILT_ROLL_Y'] * 0.1
        #print("TILT Roll Y: " + str(TILT_rollY_decode) + " °")

        PRESSURE_decode = convert_keller_pressure(ploughmeter_data['Raw-Pressure'])


        #print(repr(cryoegg_data))

        ##pt1000_temperature_reading = ConvertADCValueToTemperature(cryoegg_data['Raw-TempPT1000'])

        #print("PT1000 temperature = "+repr(pt1000_temperature_reading))

        ##pressure_reading = convert_keller_pressure(cryoegg_data['Raw-Pressure'])

        #print("Pressure = "+repr(pressure_reading))

        ##keller_temperature_reading = convert_keller_temperature(cryoegg_data['Raw-TempKeller'])

        #print("Keller temperature = "+repr(keller_temperature_reading))


        print(packet_rx_time.isoformat(timespec='milliseconds'),'{0:x}'.format(separated_packet['U-Field']), ' {}'.format(separated_packet['RSSI']), 'dBm ', '{:+06.3f}'.format(TMP117_temperature_decode), "degC",  '{:+06.3f}'.format(IMU_magX_decode), "μT", '{:+06.3f}'.format(IMU_magY_decode), "μT", '{:+06.3f}'.format(IMU_magZ_decode), "μT", '{:+06.3f}'.format(TILT_accelX_decode), "mg", '{:+06.3f}'.format(TILT_accelY_decode), "mg", '{:+06.3f}'.format(TILT_accelZ_decode), "mg", '{:+06.3f}'.format(TILT_pitchX_decode), "°", '{:+06.3f}'.format(TILT_rollY_decode), "°", '{}'.format(ploughmeter_data['Raw-Conductivity']),'{:+06.3f}'.format(PRESSURE_decode), "bar", '{}'.format(ploughmeter_data['Raw-LoadCell-Ch1']),'{}'.format(ploughmeter_data['Raw-LoadCell-Ch2']), sep=" ")

        data_string = '{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}'.format(packet_rx_time.isoformat(timespec='milliseconds'), separated_packet['U-Field'], separated_packet['RSSI'], ploughmeter_data['Raw-Temperature'],  ploughmeter_data['Raw-IMU_MAG_X'], ploughmeter_data['Raw-IMU_MAG_Y'], ploughmeter_data['Raw-IMU_MAG_Z'], ploughmeter_data['Raw-TILT_ACC_X'], ploughmeter_data['Raw-TILT_ACC_Y'], ploughmeter_data['Raw-TILT_ACC_Z'], ploughmeter_data['Raw-TILT_PITCH_X'], ploughmeter_data['Raw-TILT_ROLL_Y'], ploughmeter_data['Raw-Conductivity'], ploughmeter_data['Raw-Pressure'], ploughmeter_data['Raw-LoadCell-Ch1'], ploughmeter_data['Raw-LoadCell-Ch2'])


        ##print(packet_rx_time.isoformat(timespec='milliseconds'),'{0:x}'.format(separated_packet['U-Field']), ' {}'.format(separated_packet['RSSI']), 'dBm ', '{}'.format(cryoegg_data['Raw-Conductivity']),  '{:+06.3f}'.format(pressure_reading), "bar", '{:+06.3f}'.format(pt1000_temperature_reading), "degC", '{:+06.3f}'.format(keller_temperature_reading), "degC", sep=" ")

        ##data_string = '{},{},{},{},{},{},{},{},{},{}'.format(packet_rx_time.isoformat(timespec='milliseconds'), separated_packet['U-Field'], separated_packet['RSSI'], cryoegg_data['Raw-Conductivity'],  cryoegg_data['Raw-TempPT1000'], cryoegg_data['Raw-Pressure'], cryoegg_data['Raw-TempKeller'], pressure_reading, pt1000_temperature_reading, keller_temperature_reading)

        log_file.write(data_string)
        log_file.write("\n")
        log_file.flush() # write immediately to file in case of crash

except KeyboardInterrupt:
    log_file.close()
    rx_port.close()
    print("Finished.")
