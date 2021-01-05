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

#include "pedometer.h"

#define LOW_PASS_ORDER 5

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
extern volatile uint32_t		gWarpMenuPrintDelayMilliseconds;



int32_t compute_mean(int16_t data[], uint8_t N){
	int32_t sum = 0;
	for (int i=0; i<N; i++){
		sum += data[i];
	}
	return sum / N;
}


int32_t compute_variance(int16_t data[], int32_t mean, uint8_t N){
	int32_t sum = 0;
	int32_t data_squared;
	//SEGGER_RTT_printf(0, "\nMean = %ld \t Mean^2 = %ld\n", mean, mean*mean);
	for (int i=0; i<N; i++){
		data_squared = data[i] * data[i];
		sum = sum + (data[i] - mean)*(data[i] - mean);
		//SEGGER_RTT_printf(0, "d = %d\n", data[i]);
		//SEGGER_RTT_printf(0, "d^2 = %ld\n", data_squared);
		//SEGGER_RTT_printf(0, "Sum = %ld\n", sum);
	}
	//SEGGER_RTT_printf(0, "Sum/N = %ld\n\n", sum/N);
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
	//SEGGER_RTT_WriteString(0, "\nStarting x measurement\n");
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	//SEGGER_RTT_WriteString(0, "\nI2C Active\n");
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


void read_acceleration_distribution(uint8_t N, int32_t * z_mean, int32_t * z_var){
	//int16_t x[N];
	//int16_t y[N];
	int16_t z[N];
	
	acc_measurement measurement;
	distribution z_distribution;
	
	for (int i=0; i<N; i++){
		measurement = read_accelerometer();
		//x[i] = measurement.x;
		//y[i] = measurement.y;
		z[i] = measurement.z;
	};

	//distribution.x = generate_distribution(x, N);
	//distribution.y = generate_distribution(y, N);
	z_distribution = generate_distribution(z, N);
	*z_mean = z_distribution.mean;
	*z_var = z_distribution.variance;
}


void low_pass_filter(int32_t data[], int32_t vars[], int32_t * output, int32_t * out_var, uint8_t N){
	int32_t sum = 0;
	int32_t sum_vars = 0;
	
	for(int i=0; i<N; i++){
		sum += data[i];
		sum_vars += vars[i];
		//SEGGER_RTT_printf(0, "data: %ld, sum: %lu", data[i], sum);
	}
	
	*output = sum / N;
	*out_var = sum_vars / (N*N);
}


void rotate_array_by_one(int32_t data[], uint8_t N){
	// Takes input array and moves contents to left by 1 element. Last element is kept the same
	
	for(int i=0; i<N ; i++){
		data[i] = data[i+1];
	}
}

void print_array(int32_t data[], uint8_t N){
	SEGGER_RTT_WriteString(0, "\nARRAY: ");
	for(int i=0; i<N-1 ; i++){
		SEGGER_RTT_printf(0, "%ld, ", data[i]);
	}
	SEGGER_RTT_WriteString(0, "\n");
}

/*
void print_acc_data_array(acc_distribution data, uint8_t N){
	for (int i=0; i<N; i++){
		SEGGER_RTT_printf(0, "Index %d", i);
		print_acc_distribution(data);
	}
}
*/

void print_acc_distribution(acc_distribution dist){
	SEGGER_RTT_printf(0, "\nX\tMEAN: %ld\tVARIANCE: %lu\n", dist.x.mean, dist.x.variance);
	SEGGER_RTT_printf(0, "Y\tMEAN: %ld\tVARIANCE: %lu\n", dist.y.mean, dist.y.variance);
	SEGGER_RTT_printf(0, "Z\tMEAN: %ld\tVARIANCE: %lu\n", dist.z.mean, dist.z.variance);
}




int8_t pedometer(){
	SEGGER_RTT_WriteString(0, "Starting pedometer\n\n");
	//acc_distribution dist;
	
	uint8_t N = 8;
	int32_t low_pass;
	int32_t low_pass_var;

	int32_t low_2;
	int32_t low_2_var;
	
//	int32_t x_mean[N];
//	int32_t x_var[N];
	
//	int32_t y_mean[N];
//	int32_t y_var[N];
	
	int32_t z_mean[N];
	int32_t z_var[N];

	int32_t x_mean[8] = {400, 400, 500, 500, 600, 600, 700, 700};
	int32_t x_var[8] = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};

	int32_t y_mean[8] = {10, 20, 30, 40, 50, 60, 70, 80};
	int32_t y_var[8] = {2000, 3000, 2000, 6000, 4000, 1000, 4000, 6000};
	
	for(int i=0; i<N; i++){
		read_acceleration_distribution(10, &z_mean[i], &z_var[i]);
		SEGGER_RTT_printf(0, "Z\tMEAN: %ld\tVARIANCE: %lu\n", z_mean[i], z_var[i]);
		//print_acc_distribution(dist);
		
//		x_mean[i] = dist.x.mean;
//		x_var[i] = dist.x.variance;
		
//		y_mean[i] = dist.y.mean;
//		y_var[i] = dist.y.variance;
		
//		z_mean[i] = dist.z.mean;
//		z_var[i] = dist.z.variance;
		
		OSA_TimeDelay(1000);
	};
	
	print_array(x_mean, 8);
	rotate_array_by_one(x_mean, 8);
	print_array(x_mean, 8);
	rotate_array_by_one(x_mean, 8);
	print_array(x_mean, 8);
	
	
	//for (int i=0; i<N+1; i++){
		//SEGGER_RTT_printf(0, "\n\nShift %d\n", i);
		//print_acc_data_array(data[i], N);
//		low_pass_filter(x_mean, x_var, &low_pass, &low_pass_var, 8);
//		SEGGER_RTT_WriteString(0, "\nLow pass\n");
//		SEGGER_RTT_printf(0, "%ld\t%lu\n\n", low_pass, low_pass_var);

//		OSA_TimeDelay(500);
		
//		low_pass_filter(y_mean, y_var, &low_2, &low_2_var, 8);
//		SEGGER_RTT_WriteString(0, "\nLow pass\n");
//		SEGGER_RTT_printf(0, "%ld\t%lu\n\n", low_2, low_2_var);
		
//		low_pass_filter(z_mean, z_var, &low_pass, &low_pass_var, 8);
//		SEGGER_RTT_WriteString(0, "\nLow pass\n");
//		SEGGER_RTT_printf(0, "%ld\t%lu\n\n", low_pass, low_pass_var);
		
		//rotate_array_by_one(data, N);
	//}
	
	return 0;
}
