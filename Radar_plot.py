import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import scipy.signal as signal
import math
import time

com_port = 4
baudrate = 2000000
read_distance = 200  # Range of sensor readout in mm
packet_size = 413
resolution = 20  # plotting increments in mm

# How much much does the water fall per meter? messured in meters
pipe_slope = 0.05
# The diameter of the pipe in meters
pipe_diameter = 0.1
# The roughness of the pipe
mannings_roughness_coefficient = 0.009

# Manning Roughness Coefficient of diffrent pipe materials
"""
    Asbestos cement - 0.011
    Brass - 0.011
    Brick - 0.015
    Cast-Iron, new - 0.012
    Concrete, steel forms - 0.011
    Concrete, wooden forms - 0.015
    Concrete, centrifugally spun - 0.013
    Copper - 0.011
    Corrugated metal - 0.022
    Galvanized iron - 0.016
    Lead - 0.011
    Plastic - 0.009
    Steel, Coal-Tar Enamel - 0.01
    Steel, new unlined - 0.011
    Steel, Riveted - 0.019
"""

packet_arange = np.arange(13, packet_size, resolution)
dist_arange = np.arange(0, read_distance, (resolution / 2))

# Set up the serial connection
ser = serial.Serial(f'COM{com_port}', baudrate=baudrate)

# Initialize an empty ndarray for your integer values
data = np.array([], dtype=int)
buffer = np.array([], dtype=int)


"""
    Calculates the flow rate of the water in the pipe
    
    Args:
    flow_depth: The depth of the water in the pipe
    slope: The slope of the pipe
    roughness: The roughness of the pipe
    diameter: The diameter of the pipe
"""
def calculate_flow_rate(flow_depth, slope=pipe_slope, roughness=mannings_roughness_coefficient, diameter=pipe_diameter):
    radii = (diameter * 10) / 2

    depth_ratio = (radii - round(flow_depth, 2)) / radii

    wetted_perimeter = 2 * radii * math.acos(depth_ratio)

    # Calculate the cross-sectional area of the flow
    flow_area = radii ** 2 * (wetted_perimeter - math.sin(wetted_perimeter)) / 2

    # Calculate the hydraulic radius
    hydraulic_radius = flow_area / (radii * wetted_perimeter)

    # Calculate the flow rate
    flow_rate = (flow_area * (hydraulic_radius ** (2 / 3)) * (slope ** (1 / 2))) / roughness

    return flow_rate

last_time = time.time()
total_flow_volume = 0

"""
    Calculates the total flow volume of the water
    
    Args:
    flow_rate: The flow rate of the water
    last_time: The time of the last calculation
    total_flow_volume: The total flow volume of the water
"""
def calculate_total_flow(flow_rate, last_time, total_flow_volume=total_flow_volume):
    delta_time = time.time() - last_time
    delta_flow = flow_rate * delta_time
    total_flow_volume += delta_flow
    return total_flow_volume


# Create a function to collect data until a "." delimiter is received
def collect_data():
    global buffer, data
    line = ""

    while True:
        try:
            char = ser.read().decode()
            if char == ".":
                break
            elif char == ",":
                if line:
                    data = np.append(data, int(line))
                line = ""
            elif char.isdigit():
                line += char

            if data.size == packet_size:
                # Tryes to limit the amount of noise in the data by removing data points that are more than 100mm away from the previous and next data point
                for i in range(len(data)):
                    if i != 0 and i < len(data) - 1:
                        if data[i] > 100 + data[i - 1] and data[i] > 100 + data[i + 1]:
                            data[i] = data[i - 1] + data[i + 1] / 2
                data = signal.medfilt(data)
                buffer = np.copy(data)
                break

        except ValueError:
            pass
        

"""
    Plots the data and calculates the flow rate of the water
    
    Args:
    data: The data to be plotted
"""
def plot(data):
    try:
        water_height = (305 - np.argmax(data)) / 2
        if water_height < 0:
            water_height = 1
        print(f"Flow rate: {round(calculate_flow_rate((water_height) / 100), 2)} Liter/s ## Total flow: {round(calculate_total_flow(calculate_flow_rate((water_height) / 100), last_time), 2)} Liter(s)")
        plt.plot(data, label=f'Water level: {water_height}mm')
        plt.xticks(packet_arange, (np.flip(dist_arange)/2))
        plt.xticks(rotation=45)
    except:
        #Checks if the water height is defined, if the first plot() exception is thrown before water_height is defined it will set water_height to 0
        try:
            plt.plot(data, label=f'Water level: {water_height}mm')
            plt.xticks(packet_arange, (np.flip(dist_arange)/2))
            plt.xticks(rotation=45)
        except:
            water_height = 0
            plt.plot(data, label=f'Water level: {water_height}mm')
            plt.xticks(packet_arange, (np.flip(dist_arange)/2))
            plt.xticks(rotation=45)


# Create a function to update the plot
def animate(i):
    plt.clf()  # Clear the current plot
    global data
    data = np.array([], dtype=int)
    collect_data()
    if data.size == packet_size:

        plot(data)
    else:
        plot(buffer)

    plt.legend()
    


# Set up the figure and animation

fig, ax = plt.subplots()
ani = FuncAnimation(fig, animate, interval=5)  # Update every 1/10.th seconds

plt.show()