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

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
extern volatile uint32_t		gWarpMenuPrintDelayMilliseconds;



int32_t compute_mean(int16_t data[], int16_t N){
	int32_t sum = 0;
	for (int i=0; i<N; i++){
		sum += data[i];
	}
	return sum / N;
}


int64_t compute_variance(int16_t data[], int32_t mean, int8_t N){
	int64_t sum = 0;
	for (int i=0; i<N; i++){
		sum += data[i] * data[i];
	}
	return sum / N - mean*mean;
}


distribution generate_distribution(int16_t data[], int8_t N){
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
	SEGGER_RTT_WriteString(0, "Starting x measurement\n");
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
	
	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
	
	if (i2cReadStatus != kWarpStatusOK){
		SEGGER_RTT_WriteString(0, "x not read\n");
		measurement.x = 0;
	} else{
		SEGGER_RTT_printf(0, "x read %d\n", readSensorRegisterValueCombined);
		measurement.x = readSensorRegisterValueCombined;
	}
	
	
	/*
	 * Y axis
	 */
	SEGGER_RTT_WriteString(0, "Starting y measurement\n");
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
	
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
	
	if (i2cReadStatus != kWarpStatusOK){
		SEGGER_RTT_WriteString(0, "y not read\n");
		measurement.y = 0;
	} else{
		SEGGER_RTT_printf(0, "y read %d\n", readSensorRegisterValueCombined);
		measurement.y = readSensorRegisterValueCombined;
	}
	
	
	/*
	 * Z axis
	 */
	SEGGER_RTT_WriteString(0, "Starting z measurement\n");
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
	
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
	
	if (i2cReadStatus != kWarpStatusOK){
		SEGGER_RTT_WriteString(0, "z not read\n");
		measurement.z = 0;
	} else{
		SEGGER_RTT_printf(0, "z read %d\n", readSensorRegisterValueCombined);
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


int8_t pedometer(){
	SEGGER_RTT_WriteString(0, "Starting pedometer\n\n");
	acc_distribution dist;
	for(int i=0; i<1000; i++){
		dist = read_acceleration_distribution(10);
		SEGGER_RTT_printf(0, "\nMEAN X: %ld \t VAR X: %lld\n", dist.x.mean, dist.x.variance);
		SEGGER_RTT_printf(0, "MEAN Y: %ld \t VAR Y: %lld\n", dist.y.mean, dist.y.variance);
		SEGGER_RTT_printf(0, "MEAN Z: %ld \t VAR Z: %lld\n\n\n\n", dist.z.mean, dist.z.variance);
		OSA_TimeDelay(1000);
	};
	
	return 0;
}
