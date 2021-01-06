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
#define SAMPLE_WINDOW 50

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
	int16_t data_squared;
	//SEGGER_RTT_printf(0, "\nMean = %d \t Mean^2 = %d\n", mean, mean*mean);
	for (int i=0; i<N; i++){
		data_squared = data[i] * data[i];
		sum = sum + (data[i] - mean)*(data[i] - mean);
		//SEGGER_RTT_printf(0, "d = %d\n", data[i]);
		//SEGGER_RTT_printf(0, "d^2 = %d\n", data_squared);
		//SEGGER_RTT_printf(0, "Sum = %d\n", sum);
	}
	//SEGGER_RTT_printf(0, "Sum/N = %d\n\n", sum/N);
	return sum / N;
}


distribution generate_distribution(int16_t data[], uint8_t N){
	distribution new_dist;
	new_dist.mean = compute_mean(data, N);
	new_dist.variance = compute_variance(data, new_dist.mean, N);
	return new_dist;
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


acc_distribution read_acceleration_distribution(uint8_t N){
	int16_t x[N];
	int16_t y[N];
	int16_t z[N];
	
	acc_measurement measurement;
	acc_distribution distribution;
	
	for (int i=0; i<N; i++){
		measurement = read_accelerometer();
		x[i] = measurement.x;
		y[i] = measurement.y;
		z[i] = measurement.z;
	};

	distribution.x = generate_distribution(x, N);
	distribution.y = generate_distribution(y, N);
	distribution.z = generate_distribution(z, N);
	
	return distribution;
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


void print_acc_distribution(acc_distribution dist){
	SEGGER_RTT_printf(0, "\nX\tMEAN: %d\tVARIANCE: %d\n", dist.x.mean, dist.x.variance);
	SEGGER_RTT_printf(0, "Y\tMEAN: %d\tVARIANCE: %d\n", dist.y.mean, dist.y.variance);
	SEGGER_RTT_printf(0, "Z\tMEAN: %d\tVARIANCE: %d\n", dist.z.mean, dist.z.variance);
}




int8_t pedometer(){
	SEGGER_RTT_WriteString(0, "Starting pedometer\n\n");
	acc_distribution dist;
	
	int8_t count = 0;
	
	
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
	
	
	int16_t max_x[2];
	int16_t min_x[2];
	
	int16_t max_y[2];
	int16_t min_y[2];
	
	int16_t max_y[2];
	int16_t min_y[2];
	
	
	
	stats current_stats;
	stats new_stats;
	
	
	
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
		dist = read_acceleration_distribution(10);
		print_acc_distribution(dist);
		
		x_mean[LOW_PASS_ORDER-1] = dist.x.mean;
		x_var[LOW_PASS_ORDER-1] = dist.x.variance;
		
		y_mean[LOW_PASS_ORDER-1] = dist.y.mean;
		y_var[LOW_PASS_ORDER-1] = dist.y.variance;
		
		z_mean[LOW_PASS_ORDER-1] = dist.z.mean;
		z_var[LOW_PASS_ORDER-1] = dist.z.variance;
		
		
		// Filter current data array
		low_pass_filter(x_mean, x_var, LOW_PASS_ORDER, &low_pass_x[1], &low_pass_var_x[1]);
		low_pass_filter(y_mean, y_var, LOW_PASS_ORDER, &low_pass_y[1], &low_pass_var_y[1]);
		low_pass_filter(z_mean, z_var, LOW_PASS_ORDER, &low_pass_z[1], &low_pass_var_z[1]);
		
		
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
		
		
		
		print_array(x_mean, LOW_PASS_ORDER);
		print_array(x_var, LOW_PASS_ORDER);
		SEGGER_RTT_printf(0, "op: %d, error: %d\n\n\n", low_pass_x[1], low_pass_var_x[1]);
		OSA_TimeDelay(100);
		
		print_array(y_mean, LOW_PASS_ORDER);
		print_array(y_var, LOW_PASS_ORDER);
		OSA_TimeDelay(100);
		SEGGER_RTT_printf(0, "op: %d, error: %d\n\n\n", low_pass_y[1], low_pass_var_y[1]);
		
		print_array(z_mean, LOW_PASS_ORDER);
		print_array(z_var, LOW_PASS_ORDER);
		SEGGER_RTT_printf(0, "op: %d, error: %d\n\n\n", low_pass_z[1], low_pass_var_z[1]);
		OSA_TimeDelay(100);
		
		print_stats(new_stats);
		
		OSA_TimeDelay(700);
	};


	return 0;
}
