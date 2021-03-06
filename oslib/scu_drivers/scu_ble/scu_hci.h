/**
 ******************************************************************************
 * @file scu_hci.h
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains the required definitions by scu_hci.c
 ******************************************************************************
 **/

#ifndef SCU_HCI
#define SCU_HCI

/* Define Device IDs (DIDs) */
#define HTS221_TEMPERATURE 1
#define HTS221_HUMIDITY 2
#define LPS22HB_AIR_PRESSURE 3
#define CCS811_VOC 4
#define CCS811_CO2 5
#define LIS2DH_X_ACCELERATION 6
#define LIS2DH_Y_ACCELERATION 7
#define LIS2DH_Z_ACCELERATION 8
#define RGB_LED 9
#define BUZZER 10
#define PUSHBUTTON 11
#define ULTRASONIC 12

struct __attribute__((packed)) HCI_Message {
	uint8_t preamble;
	uint8_t type;
	uint8_t len;
	// Check that did is in the right order lol
	uint8_t did;
	uint8_t data[15];
	
};

// Function prototypes
int hci_command_interpret(struct HCI_Message *, struct HCI_Message *);
#endif