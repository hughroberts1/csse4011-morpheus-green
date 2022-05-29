/**
 ******************************************************************************
 * @file scu_.h
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains the required definitions by scu_power_management.c
 ******************************************************************************
 **/
#ifndef SCU_POWER_MANAGEMENT
#define SCU_POWER_MANAGEMENT

/* Device Tree Macros */

/* Function Prototypes..*/
int scu_power_management_init(void);

#define VDD_NODE DT_NODE_LABEL(vdd-pwr)
#define VDD DT_GPIO_LABEL(VDD_NODE, gpios)
#define VDD_PIN DT_GPIO_PIN(VDD_NODE, gpios)
#define VDD_FLAGS DT_GPIO_FLAGS(VDD_NODE, gpios)

#define CCS_VDD_NODE DT_NODE_LABEL(ccs-pwr)
#define CCS_VDD DT_GPIO_LABEL(CCS_VDD_NODE, gpios)
#define CCS_VDD_PIN DT_GPIO_PIN(CCS_VDD_NODE, gpios)
#define CCS_VDD_FLAGS DT_GPIO_FLAGS(CCS_VDD_NODE, gpios)

#endif