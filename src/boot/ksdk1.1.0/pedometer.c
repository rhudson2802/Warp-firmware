#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"

#include "pedometer.h"

#define LOW_PASS_ORDER 6		// Number of variables which are input to the low pass filter
#define SAMPLE_WINDOW 100		// Number of samples which are taken before the threshold is reset
#define SAMPLE_DELAY 10			// Time delay between samples
#define TOLERANCE 125			// Threshold the change in acceleration before a step will be registered to make the algorithm resistant to noise
#define SAMPLES_PER_DIST 10		// Number of samples included in one acceleration measurement distribution
#define MEAN 0					// Index of the mean for arrays of the form {MEAN, VARIANCE} (improves code readability)
#define VAR 1					// Index of the variance for arrays of the form {MEAN, VARIANCE} (improves code readability)

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
extern volatile uint32_t		gWarpMenuPrintDelayMilliseconds;



int16_t compute_mean(int16_t data[], uint8_t N){
	/*
	 * Computes the mean of input data array. N is the length of the array
	 */
	
	int16_t sum = 0;
	for (int i=0; i<N; i++){
		sum += data[i];
	}
	return sum / N;
}


int16_t compute_variance(int16_t data[], int16_t mean, uint8_t N){
	/*
	 * Computes the variance of input data array, given the mean. N is the length of the array
	 */
	
	int16_t sq_sum = 0;
	for (int i=0; i<N; i++){
		sq_sum = sq_sum + (data[i] - mean)*(data[i] - mean);
	}
	
	return sq_sum / N;
}


void generate_distribution(int16_t data[], uint8_t N, int16_t * mean, int16_t * variance){
	/*
	 * Takes input array data of length N, and computes the mean and variance, saving in the respective input pointers.
	 */
	
	*mean = compute_mean(data, N);
	*variance = compute_variance(data, *mean, N);
}


acc_measurement read_accelerometer(){
	/*
	 * Performs a read of the accelerometer, returning an acc_measurement struct with the x, y and z accelerations
	 */
	
	acc_measurement measurement;

	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;
	
	// The following code is taken from devMMA8451Q.c

	/*
	 * Read the three axes of the accelerometer in turn.
	 */

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	 
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	// Store the measured acceleration in the measurement struct, and save 0 if the read fails
	if (i2cReadStatus != kWarpStatusOK){
		measurement.x = 0;
	} else{
		measurement.x = readSensorRegisterValueCombined;
	}


	/*
	 * Y axis
	 */
	 
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK){
		measurement.y = 0;
	} else{
		measurement.y = readSensorRegisterValueCombined;
	}


	/*
	 * Z axis
	 */
	 
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK){
		measurement.z = 0;
	} else{
		measurement.z = readSensorRegisterValueCombined;
	}

	return measurement;
}


void read_acceleration_distribution(uint8_t N, int16_t * x_mean, int16_t * x_var, int16_t * y_mean, int16_t * y_var, int16_t * z_mean, int16_t * z_var){
	/*
	 * Performs a repeated read on the accelerometer N times, and computes the mean and variance of all three axes, saving to the respective pointers
	 */

	// Initialise variables to store the running sum of accelerations, and the running square sum. Each element of the array corresponds to a different axis (x:0, y:1, z:2)
	int32_t sum[3] = {0, 0, 0};
	int32_t sq_sum[3] = {0, 0, 0};

	// Declare measurement struct
	acc_measurement measurement;

	// Take N readings
	for (int i=0; i<N; i++){
		measurement = read_accelerometer();
		
		// Add the current measurements to the running sums for all three axes
		sum[0] = sum[0] + measurement.x;
		sq_sum[0] = sq_sum[0] + measurement.x * measurement.x;

		sum[1] = sum[1] + measurement.y;
		sq_sum[1] = sq_sum[1] + measurement.y * measurement.y;

		sum[2] = sum[2] + measurement.z;
		sq_sum[2] = sq_sum[2] + measurement.z * measurement.z;
	};

	// Check if the squared sum has overflowed. If it has, reset it to the maximum value a signed 32 bit integer can store
	for (int i=0; i<3; i++){
		if (sq_sum[i] < 0){
			sq_sum[i] = INT32_MAX;
		}
		
		// Compute E(x^2) = square sum / N
		sq_sum[i] = sq_sum[i] / N;
	}

	// Compute means and variances
	*x_mean = sum[0] / N;
	// sq_sum is now E(x^2), so this is the variance formula var(x) = E(x^2) - E(x)^2
	*x_var = sq_sum[0] - (*x_mean * *x_mean);

	*y_mean = sum[1] / N;
	*y_var = sq_sum[1] - (*y_mean * *y_mean);

	*z_mean = sum[2] / N;
	*z_var = sq_sum[2] - (*z_mean * *z_mean);

	
	// Again check if the variances have overflowed, and reset to the maximum 16 bit signed integer if they have
	if (*x_var < 0){
		*x_var = INT16_MAX;
	}
	if (*y_var < 0){
		*y_var = INT16_MAX;
	}
	if (*z_var < 0){
		*z_var = INT16_MAX;
	}
}


void rotate_array_by_one(int16_t data[], uint8_t N){
	/*
	 * Takes input array and moves contents to left by 1 element. Last element is kept the same
	 */

	for(int i=0; i<N-1 ; i++){
		data[i] = data[i+1];
	}
}


void print_array(int16_t data[], uint8_t N){
	/*
	 * Prints the entire contents of array data with length N
	 */
	
	SEGGER_RTT_WriteString(0, "\nARRAY: ");
	for(int i=0; i<N ; i++){
		SEGGER_RTT_printf(0, "%d, ", data[i]);
	}
	SEGGER_RTT_WriteString(0, "\n");
}


void low_pass_filter(int16_t means[], int16_t vars[], uint8_t N, int16_t output[]){
	/*
	 * Computes the output of a low pass filter of data in means. As each data point is equally weighted, this is equivalent to computing the mean of the data. 
	 * The propagated uncertainty is also calculated from the input uncertainty array vars
	 * Again, N is the length of the array, and the results will be stored in output.
	 * output array has form {MEAN, VARIANCE}
	 */
	
	int32_t sum_mean;
	int32_t sum_vars;

	sum_mean = 0;
	sum_vars = 0;

	// Compute running sum for the means and vars arrays
	for (int i=0; i<N; i++){
		sum_mean = sum_mean + means[i];
		sum_vars = sum_vars + vars[i];
	}

	// Compute low pass output: SUM(means) / N
	output[MEAN] = sum_mean / N;
	
	// Compute propagated uncertainty. By uncertainty propagation equation: SUM(input uncertainty) / N^2
	output[VAR] = sum_vars / (N*N);

	// Check variance hasn't overflowed and reset it to the max 16 bit integer if it has
	if (output[VAR] < 0) {
		output[VAR] = INT16_MAX;
	}
}


void equate_arrays(int16_t input[], int16_t output[], uint8_t length){
	/*
	 * Sets all elements of output array to equal their corresponding element in input array. Both arrays must have equal length (which is an input argument)
	 */
	 
	for (int i=0; i<length; i++){
		output[i] = input[i];
	}
}




uint16_t if_variance(int16_t var1[], int16_t var2[]){
	/*
	 * Compute the uncertainty propagated by an if(var1 < var2) statement, according to the derived model
	 */
	
	int16_t uncertainty;
	
	// Transform var1 to N(0, 1), apply the same transformation to var2, and compute the difference in mean and ratio of variances of the transformed variables
	int16_t x = (var2[MEAN] - var1[MEAN] ) / var1[VAR];
	int16_t sigma = var2[VAR] / var1[VAR];

	// Compute uncertainty according to model
	uncertainty = (1 - (x*x) / ((sigma*3/100 + 1)*(sigma*3/100 + 1))) / (4 * (sigma*3/100 + 1));
	if (uncertainty < 0){
		uncertainty = 0;
	}
	
	return uncertainty;
}




int8_t pedometer(){
	SEGGER_RTT_WriteString(0, "Starting pedometer\n\n");
	OSA_TimeDelay(1000);
	
	// Count variable keeps track of when we need to reset the sampling window
	int8_t count = 1;
	
	// First run flag is set when we have passed through 1 SAMPLE_WINDOW of data, as this is when we can start counting steps
	int8_t first_run_flag = 0;


	// Declare arrays to store the mean and variance of acceleration distributions. Only data up to the order of the low pass filter needs to be stored
	int16_t x_mean[LOW_PASS_ORDER];
	int16_t x_var[LOW_PASS_ORDER];

	int16_t y_mean[LOW_PASS_ORDER];
	int16_t y_var[LOW_PASS_ORDER];

	int16_t z_mean[LOW_PASS_ORDER];
	int16_t z_var[LOW_PASS_ORDER];


	// Declare arrays to store the current value of the data which has been put through a low pass filter. The arrays have the form {MEAN, UNCERTAINTY}
	int16_t low_pass_x[2];
	int16_t low_pass_y[2];
	int16_t low_pass_z[2];
	
	// Declare array to store the previous value of low passed data for the maximum activity axis
	int16_t low_pass_old[2];


	// Initialise arrays to store maximum and minimum values of each axis for the current sampling window. The arrays have the form {MEAN, UNCERTAINTY}
	int16_t max_x[2] = {0, 0};
	int16_t min_x[2] = {0, 0};

	int16_t max_y[2] = {0, 0};
	int16_t min_y[2] = {0, 0};

	int16_t max_z[2] = {0, 0};
	int16_t min_z[2] = {0, 0};


	// Variable to store maximum activity axis. The value for each axis is - X:0, Y:1, Z:2
	uint8_t max_axis = 0;
	
	// Initialise variable to store the current threshold. Has the form {MEAN, UNCERTAINTY}
	int16_t threshold[2] = {0, 0};

	// Initialise variable to store the number of steps counted. Has the form {MEAN, UNCERTAINTY}
	uint16_t step_count[2] = {0, 0};


	// Fill arrays with initial data. This is so we have real data when we perform the first low pass filtering, so the system doesn't take a long time to settle to steady state.
	for (int i=0; i<LOW_PASS_ORDER; i++) {
		// Rotate data arrays to save new datapoint at end
		rotate_array_by_one(x_mean, LOW_PASS_ORDER);
		rotate_array_by_one(x_var, LOW_PASS_ORDER);

		rotate_array_by_one(y_mean, LOW_PASS_ORDER);
		rotate_array_by_one(y_var, LOW_PASS_ORDER);

		rotate_array_by_one(z_mean, LOW_PASS_ORDER);
		rotate_array_by_one(z_var, LOW_PASS_ORDER);

		// Save new datapoint
		read_acceleration_distribution(SAMPLES_PER_DIST, &x_mean[LOW_PASS_ORDER-1], &x_var[LOW_PASS_ORDER-1], &y_mean[LOW_PASS_ORDER-1], &y_var[LOW_PASS_ORDER-1], &z_mean[LOW_PASS_ORDER-1], &z_var[LOW_PASS_ORDER-1]);
	}



	// Low pass filter the initial data
	low_pass_filter(x_mean, x_var, LOW_PASS_ORDER, low_pass_x);
	low_pass_filter(y_mean, y_var, LOW_PASS_ORDER, low_pass_y);
	low_pass_filter(z_mean, z_var, LOW_PASS_ORDER, low_pass_z);
	
	// Set the max and min to the filtered data. This means it can be updated during the first pass of the algorithm
	equate_arrays(low_pass_x, max_x, 2);
	equate_arrays(low_pass_x, min_x, 2);
	equate_arrays(low_pass_y, max_y, 2);
	equate_arrays(low_pass_y, min_y, 2);
	equate_arrays(low_pass_z, max_z, 2);
	equate_arrays(low_pass_z, min_z, 2);



	// Main loop
	while(1){
		// Rotate data arrays to save new datapoint at end
		rotate_array_by_one(x_mean, LOW_PASS_ORDER);
		rotate_array_by_one(x_var, LOW_PASS_ORDER);

		rotate_array_by_one(y_mean, LOW_PASS_ORDER);
		rotate_array_by_one(y_var, LOW_PASS_ORDER);

		rotate_array_by_one(z_mean, LOW_PASS_ORDER);
		rotate_array_by_one(z_var, LOW_PASS_ORDER);


		// Set the value of the old low pass to the datapoint of the maximum activity axis
		if (max_axis == 0){
			equate_arrays(low_pass_x, low_pass_old, 2);
		} else if (max_axis == 1){
			equate_arrays(low_pass_y, low_pass_old, 2);
		} else{
			equate_arrays(low_pass_z, low_pass_old, 2);
		}


		// Save new datapoint
		read_acceleration_distribution(10, &x_mean[LOW_PASS_ORDER-1], &x_var[LOW_PASS_ORDER-1], &y_mean[LOW_PASS_ORDER-1], &y_var[LOW_PASS_ORDER-1], &z_mean[LOW_PASS_ORDER-1], &z_var[LOW_PASS_ORDER-1]);


		// Filter current data array
		low_pass_filter(x_mean, x_var, LOW_PASS_ORDER, low_pass_x);
		low_pass_filter(y_mean, y_var, LOW_PASS_ORDER, low_pass_y);
		low_pass_filter(z_mean, z_var, LOW_PASS_ORDER, low_pass_z);


		// Check if we have a new maximum
		// As our model says that uncertainty of the max function is simply the uncertainty of the maximum variable, we set the new max to the mean and variance of the datapoint
		// As min(x1, x2) = max(-x1, -x2), the propagated uncertainty is calculated in the same way
		if (low_pass_x[MEAN] > max_x[MEAN]){
			equate_arrays(low_pass_x, max_x, 2);

		} else if(low_pass_x[MEAN] < min_x[MEAN]){
			equate_arrays(low_pass_x, min_x, 2);
		}


		if (low_pass_y[MEAN] > max_y[MEAN]){
			equate_arrays(low_pass_y, max_y, 2);

		} else if(low_pass_y[MEAN] < min_y[MEAN]){
			equate_arrays(low_pass_y, min_y, 2);
		}


		if (low_pass_z[MEAN] > max_z[MEAN]){
			equate_arrays(low_pass_z, max_z, 2);

		} else if(low_pass_z[MEAN] < min_z[MEAN]){
			equate_arrays(low_pass_z, min_z, 2);
		}


		// Decide whether a step has been taken
		if (first_run_flag){
			if (max_axis == 0){
				// Compute uncertainty of the following if statement.
				// TODO - check order of 2nd if_variance arguments
				step_count[VAR] = step_count[VAR] + if_variance(threshold, low_pass_x) + if_variance(threshold, low_pass_old);
				
				// Check whether there has been a negative transition across the threshold between the current and old reading on the maximum activity axis
				// Also make sure this is greater than the set tolerance, so we do not count noise as steps
				if ((low_pass_x[MEAN] < threshold[MEAN]) && (low_pass_old[MEAN] > threshold[MEAN]) && (low_pass_old[MEAN] - low_pass_x[MEAN] > TOLERANCE)){
					step_count[MEAN] = step_count[MEAN] + 1;
					SEGGER_RTT_printf(0, "\n\nSTEP COUNT: %d VARIANCE: %d\n\n", step_count[MEAN], step_count[VAR]);
				}
				
				
			} else if (max_axis == 1){
				step_count[VAR] = step_count[VAR] + if_variance(threshold, low_pass_y) + if_variance(threshold, low_pass_old);
				
				if ((low_pass_y[MEAN] < threshold[MEAN]) && (low_pass_old[MEAN] > threshold[MEAN] && (low_pass_old[MEAN] - low_pass_y[MEAN] > TOLERANCE))){
					step_count[MEAN] = step_count[MEAN] + 1;
					SEGGER_RTT_printf(0, "\n\nSTEP COUNT: %d VARIANCE: %d\n\n", step_count[MEAN], step_count[VAR]);
				}
				
				
			} else{
				step_count[VAR] = step_count[VAR] + if_variance(threshold, low_pass_z) + if_variance(threshold, low_pass_old);
				
				if ((low_pass_z[MEAN] < threshold[MEAN]) && (low_pass_old[MEAN] > threshold[MEAN]) && (low_pass_old[MEAN] - low_pass_z[MEAN] > TOLERANCE)){
					step_count[MEAN] = step_count[MEAN] + 1;
					SEGGER_RTT_printf(0, "\n\nSTEP COUNT: %d VARIANCE: %d\n\n", step_count[MEAN], step_count[VAR]);
				}
			}
		}


		// When we reach the end of one sampling period, reset the maximum axis
		if (count == 0){
			if (((max_x[MEAN] - min_x[MEAN]) > (max_y[MEAN] - min_y[MEAN])) && ((max_x[MEAN] - min_x[MEAN]) > (max_z[MEAN] - min_z[MEAN]))){
				max_axis = 0;
				threshold[MEAN] = (max_x[MEAN] + min_x[MEAN]) / 2;
				
				// Compute uncertainty of new threshold according to the uncertainty propagation equation
				threshold[VAR] = (max_x[VAR] + min_x[VAR]) / 4;


			} else if (((max_y[MEAN] - min_y[MEAN]) > (max_x[MEAN] - min_x[MEAN])) && ((max_y[MEAN] - min_y[MEAN]) > (max_z[MEAN] - min_z[MEAN]))) {
				max_axis = 1;
				threshold[MEAN] = (max_y[MEAN] + min_y[MEAN]) / 2;
				threshold[VAR] = (max_y[VAR] + min_y[VAR]) / 4;


			} else{
				max_axis = 2;
				threshold[MEAN] = (max_z[MEAN] + min_z[MEAN]) / 2;
				threshold[VAR] = (max_z[VAR] + min_z[VAR]) / 4;

			}

			// Set first_run_flag, as we will certainly have passed through one sampling period now
			first_run_flag = 1;

			// Reset max and min arrays for new window. The arrays are set to the current values of each axis.
			equate_arrays(low_pass_x, max_x, 2);
			equate_arrays(low_pass_x, min_x, 2);
			equate_arrays(low_pass_y, max_y, 2);
			equate_arrays(low_pass_y, min_y, 2);
			equate_arrays(low_pass_z, max_z, 2);
			equate_arrays(low_pass_z, min_z, 2);

		}
		
		// Increment counter, and reset to 0 when we reach the end of the sampling period
		count = (count + 1) % SAMPLE_WINDOW;
		
		// Pause between readings
		OSA_TimeDelay(SAMPLE_DELAY);
	};


	return 0;
}
