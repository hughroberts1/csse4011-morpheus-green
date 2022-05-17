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
#define LIS2DH_X_ACCELERATION 5
#define LIS2DH_Y_ACCELERATION 6
#define LIS2DH_Z_ACCELERATION 7
#define RGB_LED 8
#define BUZZER 9
#define PUSHBUTTON 10
//TODO: Apparently ultrasonic needs to be 6 but its already taken
#define ULTRASONIC 11

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