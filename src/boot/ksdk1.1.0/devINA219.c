/*
	Authored 2016-2018. Phillip Stanley-Marbell, Youchao Wang.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
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


extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
extern volatile uint32_t		gWarpMenuPrintDelayMilliseconds;



i2c_status_t initINA219(i2c_device_t slave, uint16_t menuI2cPullupValue){
	/*Sets up the configuration register to 16V bus voltage range, +-40mV shunt voltage range, rest as default*/
	
	i2c_status_t		status;
	uint8_t			configuration_register[1] = {0x00};		/*Configuration register address*/
	/*Value required to be written to configuration register. Needs to be split to an array of uint8_t for I2C_DRV_MasterSendDataBlocking to handle*/
	uint8_t			configuration_value[2] = {0x01, 0x9F};
	
	enableI2Cpins(menuI2cPullupValue);

	/*Write 0x019F to configuration register*/
	status = I2C_DRV_MasterSendDataBlocking(0,
										&slave,
										configuration_register,
										1,
										configuration_value,
										2,
										gWarpI2cTimeoutMilliseconds);
										
	disableI2Cpins();
	
	return status;
}


i2c_status_t setINA219Calibration(i2c_device_t slave, uint16_t calibration_value, uint16_t menuI2cPullupValue){
	/*Writes calibration_value to calibration register*/
	
	i2c_status_t		status;
	uint8_t			calibration_register[1] = {0x05};	/*Calibration register address*/
	
	uint8_t			payload[2];
	
	/* Divide 2 byte calibration_value to 2x 1 byte to send over I2C*/
	payload[0] = (uint8_t) ((calibration_value&0xFF00) >> 8);
	payload[1] = (uint8_t) (calibration_value&0x00FF);
	
	enableI2Cpins(menuI2cPullupValue);

	/*Write calibration_value to calibration register*/
	status = I2C_DRV_MasterSendDataBlocking(0,
										&slave,
										calibration_register,
										1,
										payload,
										2,
										gWarpI2cTimeoutMilliseconds);
										
	disableI2Cpins();

	return status;
}



i2c_status_t readRegisterINA219(i2c_device_t slave, uint8_t device_register, uint8_t * i2c_buffer, uint16_t menuI2cPullupValue){
	/*Performs read of device_register of device slave, storing the results in external array i2c_buffer*/
	
	i2c_status_t	status;
	uint8_t 	address_byte[1] = {device_register};
	
	enableI2Cpins(menuI2cPullupValue);

	status = I2C_DRV_MasterReceiveDataBlocking(0,
											&slave,
											address_byte,
											1,
											i2c_buffer,
											2,
											gWarpI2cTimeoutMilliseconds);
	
	disableI2Cpins();
	
	return status;
}


void printRegisterINA219(i2c_device_t slave, uint8_t device_register, uint16_t menuI2cPullupValue){
	/*Prints value stored in device_register of device slave to screen*/
	
	/*Set up buffer to store I2C data*/
	uint8_t i2c_buffer[2];

	i2c_status_t status;

	/*Get value from register over I2C*/
	status = readRegisterINA219(slave, device_register, i2c_buffer, menuI2cPullupValue);
	
	if (status != kStatus_I2C_Success){
		SEGGER_RTT_WriteString(0, "Failed to read current register\n");
		OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
	} else{
		SEGGER_RTT_printf(0, "Register 0x%02x Value: 0x%02x%02x\n", device_register, i2c_buffer[0], i2c_buffer[1]);
		OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
	}
}



uint32_t readCurrentINA219(i2c_device_t slave, uint16_t current_LSB, uint16_t menuI2cPullupValue){
	/*Reads current register on device slave, and output current calculated by current = current_register * current_LSB. Current is output in uA*/

	i2c_status_t	status;
	uint8_t			i2c_buffer[2];			/*Buffer to store I2C data*/
	uint16_t		current_register = 0;	/*Value of current register*/

	uint32_t		current;				/*Value of current calculated*/

	/*Read current register*/
	status = readRegisterINA219(slave, 0x04, i2c_buffer, menuI2cPullupValue);

	if (status == kStatus_I2C_Success){
		/*Convert 2x uint8_t array into one uint16_t value*/
		current_register = i2c_buffer[1];
		current_register |= (i2c_buffer[0] << 8);
		
		/*Calculate current in uA*/
		current = current_register * current_LSB;
		return current;
	}
	return -1;
}
