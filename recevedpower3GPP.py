import math

def calculate_path_loss(distance, frequency):
    # Air to ground path loss model for Drone - Free Space Path Loss (FSPL)
    # Distance in meters (m) and Frequency in Hz
    # Returns path loss in dB

    c = 3e8  # Speed of light
    wavelength = c / frequency
    free_space_path_loss = 20 * math.log10(distance / wavelength)
    return free_space_path_loss

def calculate_received_power(transmit_power, path_loss, transmit_antenna_gain, receive_antenna_gain):
    # Calculate received power using the Friis transmission equation
    # Transmit Power (PT), Path Loss (PL), Transmit Antenna Gain (GT), Receive Antenna Gain (GR)
    # Returns received power in dBm

    received_power = transmit_power - path_loss + transmit_antenna_gain + receive_antenna_gain
    return received_power

# Example usage:

distance = 100  # m
frequency = 2.4e9  # Hz (for example, a typical frequency for Wi-Fi)

transmit_power = 20  # dBm (Assumed, can be changed)
transmit_antenna_gain = 2  # dBi (Assumed, can be changed)
receive_antenna_gain = 2  # dBi (Assumed, can be changed)

path_loss = calculate_path_loss(distance, frequency)
received_power = calculate_received_power(transmit_power, path_loss, transmit_antenna_gain, receive_antenna_gain)

print('Received Power: ' + str(received_power) + ' dBm')
