This microcontroller code reads from the TMF8820 / TMF8821 and reports histograms (including the reference histogram) and the distance.

The code is based on the example code given by AMS on [their website](https://ams.com/tmf8820#tab/tools) (labeled "Unified Arduino driver for TMF8820/21/28"). We modify it to do the following:
- We enable histogram reporting by default
- We change the sensor to "short range, high accuracy mode" to give higher temporal resolution to the histograms, at the expense of max range (1m instead of 3m).
- We automatically start reporting data when the serial port is opened

To learn more about the sensor, check out the sensor datasheet and host driver communication document on the [AMS website](https://ams.com/tmf8820#tab/documents).
