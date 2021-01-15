#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

#define X_BORDER 13
#define Y_BORDER 7
#define DIGIT_SIZE 25

#define COLOUR_A 0		// Blue
#define COLOUR_B 0		// Green
#define COLOUR_C 0b00111110	// Red

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}



int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x0F);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	
	
	writeCommand(kSSD1331CommandDRAWRECT);	//Enter draw rectangle mode
	writeCommand(0x00);			//Enter start column
	writeCommand(0x00);			//Enter start row
	writeCommand(0x5F);			//Enter end column
	writeCommand(0x3F);			//Enter end row
	writeCommand(0x00);			//Set outline to green (colour C red)
	writeCommand(0x3F);			// Colour B (green)
	writeCommand(0x00);			// Colour A (blue)
	writeCommand(0x00);			//Set fill to green (colour C red)
	writeCommand(0x3F);			// Colour B (green)
	writeCommand(0x00);			// Colour A (blue)
	writeCommand(0x00);			// Colour A (blue)

	return 0;
}


void draw_line(uint8_t start_x, uint8_t end_x, uint8_t start_y, uint8_t end_y){
	/*
	 * Draws a line with the corresponding start and end coordinates
	 */
	
	writeCommand(kSSD1331CommandDRAWLINE);
	
	// Set start and end coordinates
	writeCommand(start_x);
	writeCommand(start_y);
	writeCommand(end_x);
	writeCommand(end_y);
	
	// Set colour
	writeCommand(COLOUR_C);
	writeCommand(COLOUR_B);
	writeCommand(COLOUR_A);
}


void clear_screen(void){
	/*
	 * Clears screen of drawing
	 */
	
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
}


void draw_number(uint8_t number, uint8_t x, uint8_t y, uint8_t scale){
	/*
	 * Draws input number at coordinate (x, y) with width scale px
	 *
	 * 
	 * Display is modelled as a 7 segment display, with lines being drawn if they would be ON on a 7 segment representation of that number.
	 * Lines are labelled clockwise from the top, with the centre line being index 6
	 * seven_segment_section stores whether each segment should be lit or not by setting or clearing the corresponding bit
	 * 
	 *    0
	 *    --
	 *  5|  | 1
	 *    --6
	 *  4|  | 2
	 *    --
	 *    3
	*/
	
	uint8_t seven_segment_section = 0;
	
	// Set the correct bits depending on the number we want to display
	switch(number){
		case 0:{
			seven_segment_section = 0b00111111;
			break;
		}
		case 1:{
			seven_segment_section = 0b00000110;
			break;
		}
		case 2:{
			seven_segment_section = 0b01011011;
			break;
		}
		case 3:{
			seven_segment_section = 0b01001111;
			break;
		}
		case 4:{
			seven_segment_section = 0b01100110;
			break;
		}
		case 5:{
			seven_segment_section = 0b01101101;
			break;
		}
		case 6:{
			seven_segment_section = 0b01111101;
			break;
		}
		case 7:{
			seven_segment_section = 0b00000111;
			break;
		}
		case 8:{
			seven_segment_section = 0b01111111;
			break;
		}
		case 9:{
			seven_segment_section = 0b01101111;
			break;
		}
		default:{
			seven_segment_section = 0;
		}
		
	}
	
	
	// Test each bit in turn, and if it is set then draw that segment
	if ( seven_segment_section & (1<<0) ){
		draw_line(x, x+scale, y, y);
	}
	
	if ( seven_segment_section & (1<<1) ){
		draw_line(x+scale, x+scale, y, y+scale);
	}
	
	if ( seven_segment_section & (1<<2) ){
		draw_line(x+scale, x+scale, y+scale, y+2*scale);
	}
	
	if ( seven_segment_section & (1<<3) ){
		draw_line(x, x+scale, y+2*scale, y+2*scale);
	}
	
	if ( seven_segment_section & (1<<4) ){
		draw_line(x, x, y+scale, y+2*scale);
	}
	
	if ( seven_segment_section & (1<<5) ){
		draw_line(x, x, y, y+scale);
	}

	if ( seven_segment_section & (1<<6) ){
		draw_line(x, x+scale, y+scale, y+scale);
	}
	SEGGER_RTT_WriteString(0, "\n");
}


void draw_value(int16_t number){
	/*
	 * Draws the input number to fill the screen
	 */

	uint8_t tens = (number / 10) % 10;			// Compute 10s digit
	uint8_t units = number %10;					// Compute units digit

	clear_screen();
	draw_number(tens, (uint8_t)(X_BORDER + 5), (uint8_t)Y_BORDER, (uint8_t)DIGIT_SIZE);
	draw_number(units, (uint8_t)(X_BORDER + 45), (uint8_t)Y_BORDER, (uint8_t)DIGIT_SIZE);
}
