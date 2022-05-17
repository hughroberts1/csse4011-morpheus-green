#include <zephyr/types.h>
#include <stddef.h>

/* struct to hold HCI packet data */
struct __attribute__((__packed__)) HCI_Message {
	uint8_t preamble;
	uint8_t type;
	uint8_t length;
	uint8_t data[16];
	uint8_t did; 
};