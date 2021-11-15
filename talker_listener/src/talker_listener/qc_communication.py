import socket
import time
import numpy as np

CONVERSION_FACTOR = 0.000286  # conversion factor needed to get values in mV


def read_raw_bytes(connection, number_of_all_channels, bytes_in_sample):
    buffer_size = number_of_all_channels * bytes_in_sample
    new_bytes = connection.recv(buffer_size)
    return new_bytes


# Convert byte-array value to an integer value and apply two's complement
def convert_bytes_to_int(bytes_value):
    value = int.from_bytes(bytes_value, byteorder='little', signed=True)
    return value


# Convert channels from bytes to integers
def bytes_to_integers(
        sample_from_channels_as_bytes,
        number_of_channels,
        bytes_in_sample,
        output_milli_volts):
    channel_values = []
    # Separate channels from byte-string. One channel has
    # "bytes_in_sample" many bytes in it.
    for channel_index in range(number_of_channels):
        channel_start = channel_index * bytes_in_sample
        channel_end = (channel_index + 1) * bytes_in_sample
        channel = sample_from_channels_as_bytes[channel_start:channel_end]

        # Convert channel's byte value to integer
        value = convert_bytes_to_int(channel)#, bytes_in_sample)

        #MAKE SURE TO CHANGE THIS BACK^^^

        # Convert bio measurement channels to milli volts if needed
        # The 4 last channels (Auxiliary and Accessory-channels)
        # are not to be converted to milli volts
        if output_milli_volts:
            value *= CONVERSION_FACTOR
        channel_values.append(value)
    return channel_values
