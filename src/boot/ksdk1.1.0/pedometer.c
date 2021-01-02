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

extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
extern volatile uint32_t		gWarpMenuPrintDelayMilliseconds;


int compute_mean(int data[], int N){
	int sum = 0;
	for (int i=0; i<N; i++){
		sum += data[i];
	}
	return sum / N;
}

int compute_variance(int data[], int mean, int N){
	int sum = 0;
	for (int i=0; i<N; i++){
		sum += data[i] * data[i];
	}
	return sum / N - mean*mean;
}

distribution generate_distribution(int data[], int N){
	distribution new_dist;
	new_dist.mean = compute_mean(data, N);
	new_dist.variance = compute_variance(data, new_dist.mean, N);
	return new_dist;
}

int pedometer(){
	int data1[5] = {1, 2, 3, 4, 5};
	int data2[5] = {2, 4, 6, 8, 10};
	int data3[5] = {1000, 2000, 3000, 4000, 5000};
	
	distribution dist1 = generate_distribution(data1, 5);
	distribution dist2 = generate_distribution(data2, 5);
	distribution dist3 = generate_distribution(data3, 5);
	
	SEGGER_RTT_printf(0, "Dist 1 mean %d var %d", dist1.mean, dist1.variance);
	SEGGER_RTT_printf(0, "Dist 2 mean %d var %d", dist2.mean, dist2.variance);
	SEGGER_RTT_printf(0, "Dist 3 mean %d var %d", dist3.mean, dist3.variance);
	return 0;
}
