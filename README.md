# 4B25 Final Project - An Uncertainty Aware Pedometer
## Robert Hudson (rh689) - Jesus College

This project concerns the design of a pedometer which can propagate measurement uncertainty, and is based on the Warp Firmware. The main implementation of the pedometer is within `/src/boot/ksdk1.1.0/pedometer.h` and `/src/boot/ksdk1.1.0/pedometer.c`. The display drivers are written in `/src/boot/ksdk1.1.0/devSSD1331.h` and `/src/boot/ksdk1.1.0/devSSD1331.c`. The git repository can be found at [https://github.com/rhudson2802/Warp-firmware](https://github.com/rhudson2802/Warp-firmware)

To run the pedometer, simply boot the Warp Firmware, and choose option 'l'. This will setup the firmware to the required parameters (enable RUN mode, and set SSSUPPLY to 3000mV). The pedometer algorithm will then run indefinitely, and print to the screen whenever it detects a step has been taken.

### Included Files
- `rh689-coursework-5.pdf` - A report detailing the motivation behind the project and the results collected
- `rh689-git-diff.txt` - A git diff run against the original Warp Firmware repository
- `/src/boot/ksdk1.1.0/pedometer.h` and `/src/boot/ksdk1.1.0/pedometer.c` - Source code for the pedometer implementation
- `/src/boot/ksdk1.1.0/devSSD1331.h` and `/src/boot/ksdk1.1.0/devSSD1331.c` - Source code for the display driver
- `rh689-schematic.pdf` - A circuit schematic for the pedometer hardware, which can be used to build a circuit not requiring the FRDM-KL03 evaluation board
- `pedometer.py` - Python script used to simulate the pedometer algorithm and perform Monte Carlo simulations on collected data
- `max_fn_uncertainty` - Python script used to compute the uncertainty propagated through the max function and if statement, which generated data to create the uncertainty models

### Hardware Setup
In order to use the display, the Adafruit SSD1331 OLED breakout board should be connected to the FRDM-KL03 using the following connections:

| Signal Name | OLED Pin | KL03 I/O Port | FRDM-KL03 Header Number | FRDM-KL03 Pin Number |
|-------------|----------|---------------|-------------------------|----------------------|
| GND         | G        | GND           | J3                      | 7                    |
| +5V         | +        | 5V            | J3                      | 5                    |
| OCS         | OC       | PTB13         | J4                      | 5                    |
| RST         | R        | PTB0          | J2                      | 6                    |
| D/C         | DC       | PTA12         | J4                      | 3                    |
| SCK         | CK       | PTA9          | J4                      | 1                    |
| MOSI        | SI       | PTA8          | J4                      | 2                    |

A schematic has also been included (`rh689-schematic.pdf`) which details the relevant circuits which are required to implement the pedometer without using the FRDM-KL03.

### Software
There are several parameters which control the performance of the algorithm, set using the macros on lines 21-25 of `pedometer.c`. These are:

- `LOW_PASS_ORDER = 6` - This controls the number of datapoints which are input to the low pass filter. A greater value will produce a smoother output, which has the effect of reducing noise. However, this cannot be increased indefinitely, as the FRDM-KL03 quickly runs out of memory to create arrays of this length.

- `SAMPLE_WINDOW = 100` - This is the number of measurements which are taken before the dynamic threshold is reset. This should be approximately the number of samples it takes to make one step, to ensure the maximum and minimum values are correctly taken.

- `SAMPLE_DELAY = 10` - This is the time delay in milliseconds between datapoints. If this is changed, then `SAMPLE_WINDOW` will also need to be changed to account for the different number of samples per step period.

- `SAMPLES_PER_DIST = 10` - This is the number of samples which are averaged to generate a distribution of measurements. This should not be increased much more, as the measurements start to change too much with a greater averaging so the variance is unacceptably large.

