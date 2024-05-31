"""
Recover planar parameters from readings from the TMF882X sensor. Plane parameters are printed
to the console
"""

import serial
import numpy as np
import argparse
from direct_method import direct_method
from differentiable_method import differentiable_method
from util import angle_between_vecs

TMF882X_CHANNELS = 10  # 9 zones + 1 reference histogram
TMF882X_BINS = 128  # bins per histogram
TMF882X_SKIP_FIELDS = 3  # skip first 3 items in each row - histogram starts at 4th
TMF882X_IDX_FIELD = 2  # second item in each row contains the idx field


def process_raw_hists(buffer):
    if len(buffer) != 31:
        print("Buffer wrong size ({}) - measurement skipped".format(len(buffer)))
        return None

    rawSum = [[0 for _ in range(TMF882X_BINS)] for _ in range(TMF882X_CHANNELS)]

    for line in buffer:
        data = line.decode("utf-8")
        data = data.replace("\r", "")
        data = data.replace("\n", "")
        row = data.split(",")

        if len(row) > 0 and len(row[0]) > 0 and row[0][0] == "#":  # if it's a well formed line
            if row[0] == "#Raw" and len(row) == TMF882X_BINS + TMF882X_SKIP_FIELDS:
                if '' in row:
                    print("Empty entry recieved over serial - skipping and returning None")
                    return None
                # idx is the id of the histogram (e.g. 0-9 for 9 hists + calibration hist)
                idx = int(row[TMF882X_IDX_FIELD])
                if idx >= 0 and idx <= 9:
                    for hist_bin in range(TMF882X_BINS):
                        rawSum[idx][hist_bin] += int(row[TMF882X_SKIP_FIELDS + hist_bin])
                elif idx >= 10 and idx <= 19:
                    for hist_bin in range(TMF882X_BINS):
                        rawSum[idx - 10][hist_bin] += int(row[TMF882X_SKIP_FIELDS + hist_bin]) * 256
                elif idx >= 20 and idx <= 29:
                    for hist_bin in range(TMF882X_BINS):
                        rawSum[idx - 20][hist_bin] += (
                            int(row[TMF882X_SKIP_FIELDS + hist_bin]) * 256 * 256
                        )
                else:
                    print("Line read with invalid idx")

        else:
            print("Incomplete line read - measurement skipped")

    return rawSum


def process_raw_dist(buffer):
    for line in buffer:
        data = line.decode("utf-8")
        data = data.replace("\r", "")
        data = data.replace("\n", "")
        d = data.split(",")

        # if there is an #Obj tag but no info in it, then the #Obj tag is just being used
        # as a separator between histograms, so return an empty distance result
        if d[0] == "#Obj" and len(d) == 1:
            return {
                "I2C_address": 0,
                "measurement_num": 0,
                "temperature": 0,
                "num_valid_results": 0,
                "tick": 0,
                "depths_1": [0 for _ in range(9)],
                "confs_1": [0 for _ in range(9)],
                "depths_2": [0 for _ in range(9)],
                "confs_2": [0 for _ in range(9)],
            }

        if d[0] == "#Obj" and len(d) == 78:
            result = {}
            result["I2C_address"] = int(d[1])
            result["measurement_num"] = int(d[2])
            result["temperature"] = int(d[3])
            result["num_valid_results"] = int(d[4])
            result["tick"] = int(d[5])
            result["depths_1"] = [
                int(x) for x in [d[6], d[8], d[10], d[12], d[14], d[16], d[18], d[20], d[22]]
            ]
            result["confs_1"] = [
                int(x) for x in [d[7], d[9], d[11], d[13], d[15], d[17], d[19], d[21], d[23]]
            ]
            # 18 that go in between here are unused, at least in 3x3 mode
            result["depths_2"] = [
                int(x) for x in [d[42], d[44], d[46], d[48], d[50], d[52], d[54], d[56], d[58]]
            ]
            result["confs_2"] = [
                int(x) for x in [d[43], d[45], d[47], d[49], d[51], d[53], d[55], d[57], d[59]]
            ]
            # last 18 are unused, at least in 3x3 mode

            return result
    return None


def get_measurement(arduino):
    buffer = []
    frames_finished = 0  # start at -1 to throw out the first frame

    while frames_finished < 1:
        line = arduino.readline().rstrip()
        if line != "":
            buffer.append(line)
        try:
            decoded_line = line.decode("utf-8").rstrip().split(",")
            if decoded_line[0] == "#Obj":
                if len(buffer) > 1:  # if histograms were reported between #Obj (depth) measurements
                    processed_hists = process_raw_hists(buffer)
                else:
                    processed_hists = None
                processed_dists = process_raw_dist(buffer)
                if processed_hists is not None and processed_dists is not None:
                    frames_finished += 1
                buffer = []

        except UnicodeDecodeError:
            print("UnicodeDecodeError - measurement skipped")
            pass  # if you start in a weird place you get random data that can't be decoded, so just ignore
            buffer = []

    return processed_hists, processed_dists


def main(method, arduino_port, baudrate, device):
    arduino = serial.Serial(port=arduino_port, baudrate=baudrate, timeout=0.1)

    print("Arduino port: {arduino.name}, baudrate: {arduino.baudrate}")

    if method == "direct":
        while True:
            hists, _ = get_measurement(arduino)
            a, d, _ = direct_method(hists[1:])
            aoi = np.rad2deg(angle_between_vecs(a, [0, 0, 1]))
            azimuth = np.rad2deg(np.arctan2(a[1], a[0]))
            z_dist = (d / a[2]) * 100
            print(f"AoI: {aoi:.2f} deg, Azimuth: {azimuth:.2f} deg, Distance: {z_dist} cm")

    elif method == "differentiable":
        while True:
            arduino.reset_input_buffer()
            hists, _ = get_measurement(arduino)
            a, d, _ = differentiable_method(hists[1:], hists[0], device)
            aoi = np.rad2deg(angle_between_vecs(a, [0, 0, 1]))
            azimuth = np.rad2deg(np.arctan2(a[1], a[0]))
            z_dist = (d / a[2]) * 100
            print(f"AoI: {aoi:.2f} deg, Azimuth: {azimuth:.2f} deg, Distance: {z_dist} cm")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TMF882X demo")
    parser.add_argument(
        "--port", "-p", type=str, default="/dev/ttyACM0", help="Arduino serial port"
    )
    parser.add_argument(
        "--baudrate", "-b", type=int, default=1000000, help="Arduino serial baudrate"
    )
    parser.add_argument(
        "--method",
        "-m",
        type=str,
        required=True,
        default="demo",
        choices=["direct", "differentiable"],
        help="Method to recover planar parameters",
    )
    parser.add_argument(
        "--device",
        "-d",
        type=str,
        required=False,
        default="cpu",
        choices=["cpu", "cuda"],
        help="Torch device to run differentiable method on (defaults to cpu)",
    )
    args = parser.parse_args()
    main(args.method, args.port, args.baudrate, args.device)
