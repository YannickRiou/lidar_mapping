import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
import os

class LFCDLaser:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(port, baud_rate, timeout=1)
        self.rpms = 0
        self.map_points = []
        self.current_position = (0, 0)
        self.map_points_history = []
        self.total_rotation = 0  # Ajout de l'attribut pour suivre la rotation totale
        self.angle_history = {}


        # Below command is not required after firmware upgrade (2017.10)
        self.serial.write(b'b')  # start motor

        # Initialize plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.distance_plot, = self.ax.plot([], [])
        self.ax.set_rmax(4)  # Adjust the radial limit based on your Lidar range
        self.ax.set_rticks([1, 2, 3, 4])
        plt.ion()  # Turn on interactive mode

    def close(self):
        self.serial.write(b'e')  # stop motor
        self.serial.close()
        plt.ioff()  # Turn off interactive mode

    def get_new_position(self, current_position, rpms):
        new_angle = current_position[0] + rpms * 0.01
        new_position = (new_angle, current_position[1])
        return new_position
    
    def update_plot(self, distances):
        angles = np.radians(range(360))
        self.distance_plot.set_data(angles, distances)

        plt.draw()
        plt.pause(0.01)

    def poll(self):
        temp_char = 0
        start_count = 0
        got_scan = False
        raw_bytes = bytearray(2520)
        good_sets = 0
        motor_speed = 0
        distances = [0] * 360

        while not got_scan:
            temp_char = self.serial.read(1)

            if not temp_char:
                break

            temp_char = ord(temp_char)

            if start_count == 0:
                if temp_char == 0xFA:
                    start_count = 1
            elif start_count == 1:
                if temp_char == 0xA0:
                    start_count = 0
                    got_scan = True

                    raw_bytes[2:] = self.serial.read(2518)

                    for i in range(0, len(raw_bytes), 42):
                        if raw_bytes[i] == 0xFA and raw_bytes[i + 1] == (0xA0 + i // 42):
                            good_sets += 1
                            motor_speed += struct.unpack("<H", raw_bytes[i + 2 : i + 4])[0]

                            for j in range(i + 4, i + 40, 6):
                                index = 6 * (i // 42) + (j - 4 - i) // 6

                                byte0, byte1, byte2, byte3 = struct.unpack(
                                    "BBBB", raw_bytes[j : j + 4]
                                )

                                intensity = (byte1 << 8) + byte0
                                range_value = (byte3 << 8) + byte2

                                distances[359 - index] = range_value / 1000.0

                    self.rpms = motor_speed / good_sets / 10
                    self.update_plot(distances)

                    # Save distances to the binary file
                    self.save_to_binary_file(filename, distances)

    def get_full_map_points(self, averaged_distances):
        full_map_points = [(angle, distance) for angle, distance in averaged_distances.items()]
        return full_map_points

    def concat_and_display_maps(self, folder_path):
        binary_files = [f for f in os.listdir(folder_path) if f.endswith(".bin")]
        binary_files.sort()
        polar_map = {}

        for binary_file in binary_files:
            file_path = os.path.join(folder_path, binary_file)

            with open(file_path, 'rb') as file:
                file_data = file.read()

            distances = struct.unpack('<' + 'f' * (len(file_data) // 4), file_data)

            for i, distance in enumerate(distances):
                angle = i * (360 / len(distances))
                polar_map.setdefault(angle, []).append(distance)

        averaged_distances = {angle: np.mean(distances) for angle, distances in polar_map.items()}
        full_map_points = self.get_full_map_points(averaged_distances)

        self.display_polar_map(full_map_points)

    def display_polar_map(self, map_points_history=None):
        if map_points_history is None:
            map_points_history = self.map_points_history

        if not map_points_history:
            print("Aucun point dans l'historique des points de la carte.")
            return

        if not isinstance(map_points_history[0], list) or not map_points_history[0]:
            print("Format incorrect pour les points dans l'historique des points de la carte.")
            return

        # Mettre à jour la série polaire avec les nouvelles données
        if hasattr(self, 'polar_plot'):
            # Supprimer la série polaire précédente
            self.polar_plot.remove()

        # Concaténer les données en temps réel et historiques
        all_data = [point for data in map_points_history for point in data]
        angles, distances = zip(*all_data)

        # Scatter plot pour toutes les données en bleu
        self.polar_plot = plt.scatter(np.radians(angles), distances, marker='.', color='blue', label='All Data')

        plt.title("Carte Polaire du Lidar")
        plt.ylim([0, 4])
        plt.yticks([1, 2, 3, 4])
        plt.legend(loc='upper right')
        plt.draw()
        plt.pause(0.01)




    def save_to_binary_file(self, filename, distances):
        with open(filename, 'ab') as file:
            file.write(struct.pack('<' + 'f' * len(distances), *distances))

    def poll_and_save(self, filename):
        temp_char = 0
        start_count = 0
        got_scan = False
        raw_bytes = bytearray(2520)
        good_sets = 0
        motor_speed = 0
        distances = [0] * 360

        while not got_scan:
            temp_char = self.serial.read(1)

            if not temp_char:
                break

            temp_char = ord(temp_char)

            if start_count == 0:
                if temp_char == 0xFA:
                    start_count = 1
            elif start_count == 1:
                if temp_char == 0xA0:
                    start_count = 0

                    got_scan = True

                    raw_bytes[2:] = self.serial.read(2518)

                    for i in range(0, len(raw_bytes), 42):
                        if raw_bytes[i] == 0xFA and raw_bytes[i + 1] == (0xA0 + i // 42):
                            good_sets += 1
                            motor_speed += struct.unpack("<H", raw_bytes[i + 2 : i + 4])[0]

                            for j in range(i + 4, i + 40, 6):
                                index = 6 * (i // 42) + (j - 4 - i) // 6

                                byte0, byte1, byte2, byte3 = struct.unpack(
                                    "BBBB", raw_bytes[j : j + 4]
                                )

                                intensity = (byte1 << 8) + byte0
                                range_value = (byte3 << 8) + byte2

                                distances[359 - index] = range_value / 1000.0

                    self.rpms = motor_speed / good_sets / 10 if good_sets > 0 else 0
                    self.update_plot(distances)

                    # Save distances to the binary file
                    self.save_to_binary_file(filename, distances)

if __name__ == "__main__":
    port = "COM4"
    baud_rate = 230400
    folder_path = "."  # Dossier contenant les fichiers binaires
    filename = "lidar_data.bin"
    laser = LFCDLaser(port, baud_rate)

    try:
        while True:
            laser.poll_and_save(filename)
            print(f"RPMS: {laser.rpms}")
    except KeyboardInterrupt:
        # Fermer l'application et afficher la carte complète
        laser.close()
        laser.concat_and_display_maps(folder_path)
