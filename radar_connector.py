import struct
import socket

import cv2
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

mobile_platform_ip = '192.168.0.210'
mobile_platform_port = 12345

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
number_of_samples = 1

def get_radar_data():
    global s

    s.send(b"1")
    data = s.recv(4)
    data = struct.unpack('<i', data)[0]
    print("Received from server: {0}".format(data))
    num_samples = data

    i = 0
    radar_data_array = []

    radar_data_array.append(num_samples)

    while i < num_samples:
        data = s.recv(4)
        data = struct.unpack('<i', data)[0]
        print(data)
        radar_data_array.append(data)
        i = i + 1

    print(radar_data_array)
    # Plot the radar data
    plt.clf()
    plt.plot(radar_data_array)
    plt.title('Radar Data')
    plt.xlabel('Sample Index')
    plt.ylabel('Value')
    plt.show()
    plt.pause(0.1)
    return radar_data_array


def main():
    global s

    s.connect((mobile_platform_ip, mobile_platform_port))

    all_radar_data = []
    radar_dict = []

    key = ' '
    while key != 113:

        counter = 0
        while counter < number_of_samples:
            counter += 1
            radar_data = get_radar_data()
            #all_radar_data.append(radar_data)
            print(radar_data)

        key = cv2.waitKey(1)

    cv2.destroyAllWindows()
    s.send(b"0")
    s.close()


if __name__ == "__main__":
    main()
