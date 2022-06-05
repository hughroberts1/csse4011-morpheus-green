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

// Relevant Thingy52 power rails 
//Controls everything except the CCS power
#define VDD_NODE DT_NODELABEL(vdd_pwr)
// Controls just the CCS power
#define CCS_NODE DT_NODELABEL(ccs_pwr)


#endif