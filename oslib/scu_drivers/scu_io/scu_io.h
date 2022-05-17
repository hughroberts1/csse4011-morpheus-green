/**
 ***************************************************************************************************
 * @file scu_io.h
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains the functionality required to control the IO devices such as the RGB Well LED,
 * 	  buzzer and the pushbutton
 ***************************************************************************************************
 **/
#ifndef SCU_IO
#define SCU_IO

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/regulator.h>
#include <drivers/pwm.h>

/* The devicetree node identifier for the red led. */
#define LED0_NODE DT_ALIAS(led0)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN0 DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS0 DT_GPIO_FLAGS(LED0_NODE, gpios)

/* The devicetree node identifier for the green led. */

#define LED1_NODE DT_NODELABEL(led1)
#define LED1 DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN1 DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS1 DT_GPIO_FLAGS(LED1_NODE, gpios)

/* The devicetree node identifier for the blue led. */

#define LED2_NODE DT_NODELABEL(led2)
#define LED2 DT_GPIO_LABEL(LED2_NODE, gpios)
#define PIN2 DT_GPIO_PIN(LED2_NODE, gpios)
#define FLAGS2 DT_GPIO_FLAGS(LED2_NODE, gpios)

/* The devicetree node identifier for the pushbutton */

#define PB_NODE DT_ALIAS(sw0)
#define PB DT_GPIO_LABEL(PB_NODE, gpios)
#define PB_PIN DT_GPIO_PIN(PB_NODE, gpios)
#define PB_FLAGS DT_GPIO_FLAGS(PB_NODE, gpios)

/* The devicetree node identifier for the speaker */
#define PWM_NODE DT_NODELABEL(pwm0)
#define PWM_CTLR DT_PWMS_CTLR(PWM_NODE)
#define PWM_CHANNEL DT_PWMS_CHANNEL(PWM_NODE))

#if !DT_NODE_EXISTS(DT_NODELABEL(pwm0))
#error "whoops"
#endif

#define SPK_PWR_NODE DT_NODELABEL(spk_pwr)
#define SPK_PWR DT_GPIO_LABEL(SPK_PWR_NODE, enable_gpios)
#define SPK_PWR_PIN DT_GPIO_PIN(SPK_PWR_NODE, enable_gpios)
#define SPK_PWR_FLAGS DT_GPIO_FLAGS(SPK_PWR_NODE, enable_gpios)

/* Function prototypes */
int scu_io_led_init(uint8_t);
int scu_io_led_deinit(uint8_t);
int scu_io_led_on(uint8_t);
int scu_io_led_toggle(uint8_t);
int scu_io_led_off(uint8_t);

int scu_io_pb_init(void);
int scu_io_pb_deinit(void);
int scu_io_pb_pin_read(void);

int scu_io_spk_init(void);

#endif