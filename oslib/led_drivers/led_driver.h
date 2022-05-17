/**
 * @file led_driver.h
 * @author Contains required globals and defines for led_driver.c
 * @brief 
 * @version 0.1
 * @date 2022-04-04
 * @copyright Copyright (c) 2022
 */

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN		DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

/* The devicetree node identifies for the "led1" alias. */
#define LED1_NODE DT_ALIAS(led1)

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define RED_LED		DT_GPIO_LABEL(LED1_NODE, gpios)
#define RED_PIN 	DT_GPIO_PIN(LED1_NODE, gpios)
#define RED_FLAGS	DT_GPIO_FLAGS(LED1_NODE, gpios)
#else 
/* A build error here means your board isn't set up to blink an LED */
#error "Unsupported board: led1 devicetree alias is not defined"AIO_PRIO_DELTA_MAX
#define RED_LED 	""
#define RED_PIN		0
#define RED_FLAGS	0
#endif

/* The devicetree node identifies for the "led2" alias. */
#define LED2_NODE DT_ALIAS(led2)

#if DT_NODE_HAS_STATUS(LED2_NODE, okay)
#define GREEN_LED		DT_GPIO_LABEL(LED2_NODE, gpios)
#define GREEN_PIN 		DT_GPIO_PIN(LED2_NODE, gpios)
#define GREEN_FLAGS		DT_GPIO_FLAGS(LED2_NODE, gpios)
#else 
/* A build error here means your board isn't set up to blink an LED */
#error "Unsupported board: led2 devicetree alias is not defined"
#define GREEN_LED 	""
#define GREEN_PIN		0
#define GREEN_FLAGS	0
#endif

/* The devicetree node identifies for the "led3" alias. */
#define LED3_NODE DT_ALIAS(led3)

#if DT_NODE_HAS_STATUS(LED3_NODE, okay)
#define BLUE_LED		DT_GPIO_LABEL(LED3_NODE, gpios)
#define BLUE_PIN 		DT_GPIO_PIN(LED3_NODE, gpios)
#define BLUE_FLAGS		DT_GPIO_FLAGS(LED3_NODE, gpios)
#else 
/* A build error here means your board isn't set up to blink an LED */
#error "Unsupported board: led3 devicetree alias is not defined"
#define BLUE_LED 	""
#define BLUE_PIN		0
#define BLUE_FLAGS	0
#endif

#define RED 1
#define GREEN 2 
#define BLUE 3

#define ON 1
#define OFF 0

int led_module_init(void);
int led_module_deinit(void);
int led_toggle(int);
int led_on_off(int, int);