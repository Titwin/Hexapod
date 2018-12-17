/*
 * Configuration.h
 *
 * Created: 07/12/2018 13:26:10
 *  Author: Thibault-SED
 */ 


#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

/*
#define LED_Red   11 // PB7 OC1C
#define LED_Green 10 // PB6 OC1B
#define LED_Blue  9  // PB5 OC1A

#define VLX_RDY   8  // PB4

#define SHIELD_A A3	// PF4
#define SHIELD_B A2 // PF5
#define SHIELD_C A1 // PF6
#define SHIELD_D A4 // PF1
#define SHIELD_E A5 // center PF0
*/

//  Pinout
#define FORCE			0x07	// ADC7
#define V_BATT			0x00	// 0x00 on ADMUX & 1 on MUX5
#define TEMPERATURE		0x07	// 0x07 on ADMUX & 1 on MUX5


//  Status byte bitfield
#define EEPROM_LOCK     7
#define VLX_OK          6
#define SERIAL_USB      5
#define SHIELD_CONTACT  4
#define DISTANCE_THSD   3
#define FORCE_THSD      2
#define LED_MODE_A      1
#define LED_MODE_B      0

#define STATUS_WRITE_MASK 0x83


//  Register mapping
#define REG_STATE				0
#define REG_SHIELD				1
#define REG_LED_RED				2
#define REG_LED_GREEN			3
#define REG_LED_BLUE			4
#define REG_WDR_COUNT			5

#define REG_DISTANCE_H			6
#define REG_DISTANCE_L			7
#define REG_FORCE_H				8
#define REG_FORCE_L				9
#define REG_VBATT_H				10
#define REG_VBATT_L				11
#define REG_TEMPERATURE_H		12
#define REG_TEMPERATURE_L		13

#define REG_SIZE				14


#define EEPROM_ID					0
#define EEPROM_SLAVE_TYPE			1
#define EEPROM_SOFT_VERSION			2

#define EEPROM_DISTANCE_THSD_H		3
#define EEPROM_DISTANCE_THSD_L		4
#define EEPROM_FORCE_THSD_H			5
#define EEPROM_FORCE_THSD_L			6

#define EEPROM_SIZE					7


//	Board type
#define LEGBOARD 0x01

#endif /* CONFIGURATION_H_ */