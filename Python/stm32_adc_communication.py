#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#This script fits together with "sample_filter_transmit" stm32 project.
#Never use serial.readline()

import serial
import struct
import matplotlib.pyplot as plt
import collections

def init(port, baud: int = 115200):
    ser = serial.Serial(port, baud, timeout = 0.1) #0.1 is the timeout value
    return ser

def de_init(ser: serial.Serial):
    ser.close()
#%%
port = "/dev/ttyACM0" #Needs to be looked up using dmesg | grep tty
#port = "/dev/ttyUSB0" #Now we use USB to uart converter
UART_BUFFER_SIZE = 16 #Convention we use in this project
DECIMATION_FACTOR = 4
INPUT_NUMBER_COMPARISON = 3 #Only used if we are receiving heart rate
UPDATE_CONSTANT = 100
MAX_SAMPLES = 0

receive_heart_rate = False
using_decimation = False
sample_number = 0 #Used for real-time plotting
x_points = collections.deque(maxlen = 1000) #For real-time plotting
y_points = collections.deque(maxlen = 1000) #For real-time plotting
update = 0
heart_rate_counter = 0

stm = init(port)
stm.flush()
filename = "just testing.txt"


mode1 = int(input("Normal mode (1) or downsample mode (2)?: "))
match mode1:
    case 1:
        using_decimation = False
    case 2:
        using_decimation = True
stm.write(bytes([mode1]))

mode2 = int(input("No heart rate (1) or yes heart rate(2)?: "))
match mode2:
    case 1:
        receive_heart_rate = False
    case 2:
        receive_heart_rate = True
        if(using_decimation):
            INPUT_NUMBER_COMPARISON = 4
        else:
            INPUT_NUMBER_COMPARISON = 16
stm.write(bytes([mode2]))


if(using_decimation):
    UART_BUFFER_SIZE = int(16/DECIMATION_FACTOR)
    UPDATE_CONSTANT = 100/DECIMATION_FACTOR
if(receive_heart_rate):
        UART_BUFFER_SIZE = UART_BUFFER_SIZE + 1
UART_BYTE_SIZE = int(UART_BUFFER_SIZE * 4)


mode3 = int(input("Write to file (1) or plot in real time (2)?: "))
match mode3:
    case 1:
        MAX_SAMPLES = 20000
        if(using_decimation):
            MAX_SAMPLES = 20000/DECIMATION_FACTOR
    case 2:
        MAX_SAMPLES = 50000
        if(using_decimation):
            MAX_SAMPLES = 50000/DECIMATION_FACTOR
if(mode3 == 2):
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-',linewidth=0.5)
stm.write(bytes([mode3]))

input_number = 1
#For receiving multiple floats and store in a file
with open(filename, 'w') as file:
    counter = 0
    while(counter < MAX_SAMPLES):
        data = stm.read(UART_BYTE_SIZE)
        if(receive_heart_rate):
            counter = counter + UART_BUFFER_SIZE - 1 # -1 to account for the beats/minute number
        else:
            counter = counter + UART_BUFFER_SIZE
        if len(data) != UART_BYTE_SIZE:
            stm.flush()
            continue
        # Partition into chunks of 4 bytes each https://stackoverflow.com/questions/20024490/how-to-split-a-byte-string-into-separate-parts/20024864
        chunks = [data[i:i+4] for i in range(0, len(data), 4)]
        for i, chunk in enumerate(chunks):
            value = struct.unpack('f', chunk)[0] #Convert 4 bytes into their floating point representation
            if(mode3 == 1): #Mode 1 we write to a file
                file.write(f"{value}\n")
                
            if(mode3 == 2): #Mode 2 we plot in real time. This code is taken from chatGPT
                file.write(f"{value}\n")
                if(input_number <= INPUT_NUMBER_COMPARISON): #We only plot the first 4 values of each batch
                    x_points.append(sample_number)
                    y_points.append(value)
                    update = update + 1
                    sample_number = sample_number + 1
                    if(update == UPDATE_CONSTANT):
                        update = 0
                        line.set_xdata(x_points)
                        line.set_ydata(y_points)
                        ax.relim()
                        ax.autoscale_view()
                        plt.draw()
                        plt.pause(0.01)
                elif (input_number == (INPUT_NUMBER_COMPARISON + 1) and receive_heart_rate): #5th value is the heart rate
                    heart_rate_counter += 1
                    if(heart_rate_counter >= 200):
                        heart_rate_counter = 0
                        value = int(value)
                        print(f"Heart rate: {value} beats/minute")
                input_number += 1
                if(receive_heart_rate):
                    if(input_number > (INPUT_NUMBER_COMPARISON+1)):
                        input_number = 1
                else:
                    if(input_number > INPUT_NUMBER_COMPARISON):
                            input_number = 1
                    
de_init(stm)
#%%½
de_init(stm)