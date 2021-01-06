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

#define LOW_PASS_ORDER 5
#define SAMPLE_WINDOW 10

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
extern volatile uint32_t		gWarpMenuPrintDelayMilliseconds;



int16_t compute_mean(int16_t data[], uint8_t N){
	int16_t sum = 0;
	for (int i=0; i<N; i++){
		sum += data[i];
	}
	return sum / N;
}


int16_t compute_variance(int16_t data[], int16_t mean, uint8_t N){
	int16_t sum = 0;
	//SEGGER_RTT_printf(0, "\nMean = %d \t Mean^2 = %d\n", mean, mean*mean);
	for (int i=0; i<N; i++){
		sum = sum + (data[i] - mean)*(data[i] - mean);
		//SEGGER_RTT_printf(0, "d = %d\n", data[i]);
		//SEGGER_RTT_printf(0, "Sum = %d\n", sum);
	}
	//SEGGER_RTT_printf(0, "Sum/N = %d\n\n", sum/N);
	return sum / N;
}


void generate_distribution(int16_t data[], uint8_t N, int16_t * mean, int16_t * variance){
	*mean = compute_mean(data, N);
	*variance = compute_variance(data, *mean, N);
}


acc_measurement read_accelerometer(){
	acc_measurement measurement;

	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

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
	//SEGGER_RTT_WriteString(0, "Starting x measurement\n");
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	//SEGGER_RTT_WriteString(0, "I2C Active");
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK){
		//SEGGER_RTT_WriteString(0, "x not read\n");
		measurement.x = 0;
	} else{
		//SEGGER_RTT_printf(0, "x read %d\n", readSensorRegisterValueCombined);
		measurement.x = readSensorRegisterValueCombined;
	}


	/*
	 * Y axis
	 */
	//SEGGER_RTT_WriteString(0, "Starting y measurement\n");
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK){
		//SEGGER_RTT_WriteString(0, "y not read\n");
		measurement.y = 0;
	} else{
		//SEGGER_RTT_printf(0, "y read %d\n", readSensorRegisterValueCombined);
		measurement.y = readSensorRegisterValueCombined;
	}


	/*
	 * Z axis
	 */
	//SEGGER_RTT_WriteString(0, "Starting z measurement\n");
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK){
		//SEGGER_RTT_WriteString(0, "z not read\n");
		measurement.z = 0;
	} else{
		//SEGGER_RTT_printf(0, "z read %d\n", readSensorRegisterValueCombined);
		measurement.z = readSensorRegisterValueCombined;
	}

	return measurement;
}


void read_acceleration_distribution(uint8_t N, int16_t * x_mean, int16_t * x_var, int16_t * y_mean, int16_t * y_var, int16_t * z_mean, int16_t * z_var){
	int16_t x[N];
	int16_t y[N];
	int16_t z[N];

	acc_measurement measurement;

	for (int i=0; i<N; i++){
		measurement = read_accelerometer();
		x[i] = measurement.x;
		y[i] = measurement.y;
		z[i] = measurement.z;
	};

	generate_distribution(x, N, x_mean, x_var);
	generate_distribution(y, N, y_mean, y_var);
	generate_distribution(z, N, z_mean, z_var);
}


void rotate_array_by_one(int16_t data[], uint8_t N){
	// Takes input array and moves contents to left by 1 element. Last element is kept the same

	for(int i=0; i<N-1 ; i++){
		data[i] = data[i+1];
	}
}


void print_array(int16_t data[], uint8_t N){
	SEGGER_RTT_WriteString(0, "\nARRAY: ");
	for(int i=0; i<N ; i++){
		SEGGER_RTT_printf(0, "%d, ", data[i]);
	}
	SEGGER_RTT_WriteString(0, "\n");
}


void low_pass_filter(int16_t means[], int16_t vars[], uint8_t N, int16_t * output, int16_t * uncertainty){
	int16_t sum_mean;
	int16_t sum_vars;

	sum_mean = 0;
	sum_vars = 0;

	for (int i=0; i<N; i++){
		sum_mean = sum_mean + means[i];
		sum_vars = sum_vars + vars[i];
	}

	*output = sum_mean / N;
	*uncertainty = sum_vars / (N*N);
}


void equate_arrays(int16_t input[], int16_t output[], uint8_t length){
	for (int i=0; i<length; i++){
		output[i] = input[i];
	}
}




int8_t pedometer(){
	SEGGER_RTT_WriteString(0, "Starting pedometer\n\n");

	int8_t count = 1;
	// First run flag is set when we have passed through 1 SAMPLE_WINDOW of data, as this is when we can start counting steps
	int8_t first_run_flag = 0;



	int16_t x_mean[LOW_PASS_ORDER];
	int16_t x_var[LOW_PASS_ORDER];

	int16_t y_mean[LOW_PASS_ORDER];
	int16_t y_var[LOW_PASS_ORDER];

	int16_t z_mean[LOW_PASS_ORDER];
	int16_t z_var[LOW_PASS_ORDER];



	int16_t low_pass_x[2];
	int16_t low_pass_var_x[2];

	int16_t low_pass_y[2];
	int16_t low_pass_var_y[2];

	int16_t low_pass_z[2];
	int16_t low_pass_var_z[2];



	int16_t max_x[2] = {0, 0};
	int16_t min_x[2] = {0, 0};

	int16_t max_y[2] = {0, 0};
	int16_t min_y[2] = {0, 0};

	int16_t max_z[2] = {0, 0};
	int16_t min_z[2] = {0, 0};


	axis max_axis;
	int16_t threshold[2] = {0, 0}; 		//	 Form: threshold, uncertainty.



	// Fill arrays with initial data
	for (int i=0; i<LOW_PASS_ORDER; i++) {
		// Rotate data arrays to save new datapoint at end
		rotate_array_by_one(x_mean, LOW_PASS_ORDER);
		rotate_array_by_one(x_var, LOW_PASS_ORDER);

		rotate_array_by_one(y_mean, LOW_PASS_ORDER);
		rotate_array_by_one(y_var, LOW_PASS_ORDER);

		rotate_array_by_one(z_mean, LOW_PASS_ORDER);
		rotate_array_by_one(z_var, LOW_PASS_ORDER);

		// Save new datapoint
		read_acceleration_distribution(10, &x_mean[LOW_PASS_ORDER-1], &x_var[LOW_PASS_ORDER-1], &y_mean[LOW_PASS_ORDER-1], &y_var[LOW_PASS_ORDER-1], &z_mean[LOW_PASS_ORDER-1], &z_var[LOW_PASS_ORDER-1]);
	}



	// Initialise max and min arrays
	low_pass_filter(x_mean, x_var, LOW_PASS_ORDER, &low_pass_x[1], &low_pass_var_x[1]);
	low_pass_filter(y_mean, y_var, LOW_PASS_ORDER, &low_pass_y[1], &low_pass_var_y[1]);
	low_pass_filter(z_mean, z_var, LOW_PASS_ORDER, &low_pass_z[1], &low_pass_var_z[1]);
	
	max_x[0] = low_pass_x[1];
	max_x[1] = low_pass_var_x[1];
	min_x[0] = low_pass_x[1];
	min_x[1] = low_pass_var_x[1];
	max_y[0] = low_pass_y[1];
	max_y[1] = low_pass_var_y[1];
	min_y[0] = low_pass_y[1];
	min_y[1] = low_pass_var_y[1];
	max_z[0] = low_pass_z[1];
	max_z[1] = low_pass_var_z[1];
	min_z[0] = low_pass_z[1];
	min_z[0] = low_pass_var_z[1];



	// Main loop
	for(int i=0; i<1000; i++){
		// Rotate data arrays to save new datapoint at end
		rotate_array_by_one(x_mean, LOW_PASS_ORDER);
		rotate_array_by_one(x_var, LOW_PASS_ORDER);

		rotate_array_by_one(y_mean, LOW_PASS_ORDER);
		rotate_array_by_one(y_var, LOW_PASS_ORDER);

		rotate_array_by_one(z_mean, LOW_PASS_ORDER);
		rotate_array_by_one(z_var, LOW_PASS_ORDER);


		// Rotate low pass arrays
		low_pass_x[0] = low_pass_x[1];
		low_pass_var_x[0] = low_pass_var_x[1];

		low_pass_y[0] = low_pass_y[1];
		low_pass_var_y[0] = low_pass_var_y[1];

		low_pass_y[0] = low_pass_y[1];
		low_pass_var_y[0] = low_pass_var_y[1];


		// Save new datapoint
		read_acceleration_distribution(10, &x_mean[LOW_PASS_ORDER-1], &x_var[LOW_PASS_ORDER-1], &y_mean[LOW_PASS_ORDER-1], &y_var[LOW_PASS_ORDER-1], &z_mean[LOW_PASS_ORDER-1], &z_var[LOW_PASS_ORDER-1]);

		SEGGER_RTT_printf(0, "\nX\tMEAN: %d\tVARIANCE: %d\n", x_mean[LOW_PASS_ORDER-1], x_var[LOW_PASS_ORDER-1]);
		SEGGER_RTT_printf(0, "Y\tMEAN: %d\tVARIANCE: %d\n", y_mean[LOW_PASS_ORDER-1], y_var[LOW_PASS_ORDER-1]);
		SEGGER_RTT_printf(0, "Z\tMEAN: %d\tVARIANCE: %d\n", z_mean[LOW_PASS_ORDER-1], z_var[LOW_PASS_ORDER-1]);


		// Filter current data array
		low_pass_filter(x_mean, x_var, LOW_PASS_ORDER, &low_pass_x[1], &low_pass_var_x[1]);
		low_pass_filter(y_mean, y_var, LOW_PASS_ORDER, &low_pass_y[1], &low_pass_var_y[1]);
		low_pass_filter(z_mean, z_var, LOW_PASS_ORDER, &low_pass_z[1], &low_pass_var_z[1]);
		
		
		

		// Check if we have a new maximum
		if (low_pass_x[1] > max_x[0]){
			max_x[0] = low_pass_x[1];
			max_x[1] = low_pass_var_x[1];

		} else if(low_pass_x[1] < min_x[0]){
			min_x[0] = low_pass_x[1];
			min_x[1] = low_pass_var_x[1];
		}


		if (low_pass_y[1] > max_y[0]){
			max_y[0] = low_pass_y[1];
			max_y[1] = low_pass_var_y[1];

		} else if(low_pass_y[1] < min_y[0]){
			min_y[0] = low_pass_y[1];
			min_y[1] = low_pass_var_y[1];
		}


		if (low_pass_z[1] > max_z[0]){
			max_z[0] = low_pass_z[1];
			max_z[1] = low_pass_var_z[1];

		} else if(low_pass_z[1] < min_z[0]){
			min_z[0] = low_pass_z[1];
			min_z[0] = low_pass_var_z[1];
		}


		if (count == 0){
			if ((max_x[0] - min_x[0] > max_y[0] - min_y[0]) && (max_x[0] - min_x[0] > max_z[0] - min_z[0])){
				SEGGER_RTT_WriteString(0, "X is max axis");
				max_axis = X;
				threshold[0] = (max_x[0] - min_x[0]) / 2;
				threshold[1] = (max_x[1] + min_x[0]) / 4;

			} else if ((max_y[0] - min_y[0] > max_x[0] - min_x[0]) && (max_y[0] - min_y[0] > max_z[0] - min_z[0])) {
				SEGGER_RTT_WriteString(0, "Y is max axisl");
				max_axis = Y;
				threshold[0] = (max_y[0] - min_y[0]) / 2;
				threshold[1] = (max_y[1] + min_y[0]) / 4;

			} else{
				SEGGER_RTT_WriteString(0, "Z is max axis");
				max_axis = Z;
				threshold[0] = (max_z[0] - min_z[0]) / 2;
				threshold[1] = (max_z[1] + min_z[0]) / 4;
				
			}
			SEGGER_RTT_printf(0, "MAX AXIS: %d\t THRESHOLD: %d\t UNCERTAINTY: %d\n\n", max_axis, threshold[0], threshold[1]);
			first_run_flag = 1;

		}



		//print_array(x_mean, LOW_PASS_ORDER);
		//print_array(x_var, LOW_PASS_ORDER);
		//SEGGER_RTT_printf(0, "op: %d, error: %d\n\n\n", low_pass_x[1], low_pass_var_x[1]);
		OSA_TimeDelay(100);

		//print_array(y_mean, LOW_PASS_ORDER);
		//print_array(y_var, LOW_PASS_ORDER);
		//SEGGER_RTT_printf(0, "op: %d, error: %d\n\n\n", low_pass_y[1], low_pass_var_y[1]);
		OSA_TimeDelay(100);

		//print_array(z_mean, LOW_PASS_ORDER);
		//print_array(z_var, LOW_PASS_ORDER);
		//SEGGER_RTT_printf(0, "op: %d, error: %d\n\n\n", low_pass_z[1], low_pass_var_z[1]);
		OSA_TimeDelay(100);

		SEGGER_RTT_printf(0, "X\tMAX: %d\tMIN: %d\n", max_x[0], min_x[0]);
		SEGGER_RTT_printf(0, "Y\tMAX: %d\tMIN: %d\n", max_y[0], min_y[0]);
		SEGGER_RTT_printf(0, "Y\tMAX: %d\tMIN: %d\n", max_z[0], min_z[0]);


		count = (count + 1) % SAMPLE_WINDOW;
		OSA_TimeDelay(700);
	};


	return 0;
}
