# 4B25 Final Project - An Uncertainty Aware Pedometer
## Robert Hudson (rh689) - Jesus College

This project concerns the design of a pedometer which can propagate measurement uncertainty, and is based on the Warp Firmware. The main implementation of the pedometer is within `/src/boot/ksdk1.1.0/pedometer.h` and `/src/boot/ksdk1.1.0/pedometer.c`. The git repository can be found at [https://github.com/rhudson2802/Warp-firmware](https://github.com/rhudson2802/Warp-firmware)

To run the pedometer, simply boot the Warp Firmware, and choose option 'l'. This will setup the firmware to the required parameters (enable RUN mode, and set SSSUPPLY to 3000mV). The pedometer algorithm will then run indefinitely, and print to the screen whenever it detects a step has been taken.

There are several parameters which control the performance of the algorithm, set using the macros on lines 21-25 of `pedometer.c`. These are:

- `LOW_PASS_ORDER = 6` - This controls the number of datapoints which are input to the low pass filter. A greater value will produce a smoother output, which has the effect of reducing noise. However, this cannot be increased indefinitely, as the FRDM-KL03 quickly runs out of memory to create arrays of this length.

- `SAMPLE_WINDOW = 100` - This is the number of measurements which are taken before the dynamic threshold is reset. This should be approximately the number of samples it takes to make one step, to ensure the maximum and minimum values are correctly taken.

- `SAMPLE_DELAY = 10` - This is the time delay in milliseconds between datapoints. If this is changed, then `SAMPLE_WINDOW` will also need to be changed to account for the different number of samples per step period.

- `SAMPLES_PER_DIST = 10` - This is the number of samples which are averaged to generate a distribution of measurements. This should not be increased much more, as the measurements start to change too much with a greater averaging so the variance is unacceptably large.