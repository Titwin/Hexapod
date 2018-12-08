
//  Pinout
#define LED_Red   11
#define LED_Green 10
#define LED_Blue  9

#define VLX_RDY   8

#define SHIELD_A A3
#define SHIELD_B A2
#define SHIELD_C A1
#define SHIELD_D A4
#define SHIELD_E A5 //shield center

#define FORCE    A0
#define VBATT     4


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
#define REG_STATE           0
#define REG_DISTANCE_H      1
#define REG_DISTANCE_L      2
#define REG_FORCE_H         3
#define REG_FORCE_L         4
#define REG_SHIELD          5
#define REG_LED_RED         6
#define REG_LED_GREEN       7
#define REG_LED_BLUE        8
#define REG_SIZE            9

#define EEPROM_ID               0
#define EEPROM_SLAVE_TYPE       1
#define EEPROM_VERSION          2
#define EEPROM_DISTANCE_THSD_H  3
#define EEPROM_DISTANCE_THSD_L  4
#define EEPROM_FORCE_THSD_H     5
#define EEPROM_FORCE_THSD_L     6
#define EEPROM_FORCE_OFFSET_H   7
#define EEPROM_FORCE_OFFSET_L   8
#define EEPROM_SIZE             9

#define LEGBOARD 0x01

