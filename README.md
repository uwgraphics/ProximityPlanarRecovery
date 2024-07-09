# Proximity Sensor Planar Recovery

Demo code for our paper, "Unlocking the Performance of Proximity Sensors by Utilizing Transient Histograms"
<br>
[project page](https://cpsiff.github.io/unlocking_proximity_sensors/index.html) / [video](https://www.youtube.com/watch?v=vJdfpmd6OE0) / [pdf](https://arxiv.org/pdf/2308.13473.pdf)

# General Info
This repo contains example code for recovering planar geometry from the AMS TMF8820/TMF8821 proximity sensor. This is meant to be a demo of our method, not a standalone library. Feel free to modify and re-use this code as you see fit in accordance with the MIT license.

This code is written to work with the AMS TMF8820 or TMF8821 sensor (we only use 3x3 zones, so there is no benefit from the 4x4 zone 8821). We use the [sensor breakout board from SparkFun](https://www.sparkfun.com/products/19036) along with a [SparkFun Qwiic Pro Micro](https://www.sparkfun.com/products/15795), connected via a Qwiic cable. Any breakout board and arduino compatible microcontroller should work.

Before you plan to use this in your application, please see the "Known Limitations" section near the bottom of this README.

# Microcontroller Setup
1. Connect the TMF882X to your Arduino-compatible microcontroller, and connect the microcontroller to your computer
2. Open the arduino/arduino.ino sketch in the Arduino IDE and flash it to the microcontroller. Now, if you open the Arduino serial port, you should see measurements streaming over it
3. Take note of the port on your computer that the microcontroller is connected to (something like `/dev/ttyACM0` on Linux or `COM1` on Windows)

For more information on what the Arduino code does, see the README in the `arduino` folder.

# Python Setup
1. If you want to use the differentiable method, you will need PyTorch. For this we recommend using a conda environment. First install PyTorch in the environment, then continue to the next step. If you will not use the differentiable method, continue to the next step without installing PyTorch.
2. Install all other dependencies. If you are using conda, you can run the below command. These dependencies can also be installed via pip.
```
conda install numpy pyserial scipy matplotlib
```

# Use
1. The demo script supports two methods, "direct", which is the "peak finding - calibrated" method in the paper, and "differentiable" which is the "differentiable rendering" method in the paper.
2. Run the demo script, specifying the method to use and serial port for the Arduino, e.g.
```
python demo.py --method direct --port /dev/ttyACM0
```
3. The parameters of the planar surface - distance, slope, azimuth, and (with differentiable method) albedo will print to console as measurements arrive.

## Known Limitations
- **Frame Rate**: The frame rate of data transfer between the sensor and Arduino is limited to about 3FPS because of the I2C interface used and the sub-optimal encoding used for the transient histograms. The direct method keeps up with this sensor frame rate. The differentiable method typically does not, although with some optimization (e.g. vectorization and re-using previous estimates as a starting point) we believe it could.
- **Ambient Light**: The results in the paper show accuracy under bright indoor lights. Performance in direct sunlight will likely be lower due to the lower SNR from high ambient light rejection.
- **Materials**: Does not work well on highly specular materials. See paper for more details.
- **Sensor Variation**: The parameters for both the differentiable and direct method were calibrated on our sensor. We have subjectively observed some variation between sensors which may mean that performance is worse for sensors other than the one used for calibration.

## Troubleshooting
- If you get an error that says the serial port is unavailable or busy, you may have it open in another program, like the Arduino IDE. Be sure to close the serial port monitor before you run the demo.
- If you get a permission denied error for the serial port on Linux, you need to chmod the directory to give read permission.
- Sometimes the flash to the TMF882X fails silently. When this happens, you'll get nonsensical readings over the serial port. Sometimes its helpful to power off the sensor (so that it loses the firmware that has been flashed to its RAM) before trying to re-flash
- If the above does not solve your problem, feel free to create a GitHub issue on this repository.


--------
If you use this repo for published work, please cite our paper:
```
@article{Sifferman2023,
  author={Sifferman, Carter and Wang, Yeping and Gupta, Mohit and Gleicher, Michael},
  journal={IEEE Robotics and Automation Letters}, 
  title={Unlocking the Performance of Proximity Sensors by Utilizing Transient Histograms}, 
  year={2023},
  volume={8},
  number={10},
  pages={6843-6850},
  doi={10.1109/LRA.2023.3313069}
}
```