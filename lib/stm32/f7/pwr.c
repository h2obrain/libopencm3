/** @defgroup pwr_file PWR peripheral API
 *
 * @ingroup peripheral_apis
 *
 * @brief <b>libopencm3 STM32F7xx Power Control</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2011 Stephen Caudle <scaudle@doceme.com>
 * @author @htmlonly &copy; @endhtmlonly 2017 Matthew Lai <m@matthewlai.ca>
 *
 * @date 12 March 2017
 *
 * This library supports the power control system for the
 * STM32F7 series of ARM Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms @ref lgpl_license
 */
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2017 Matthew Lai <m@matthewlai.ca>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/pwr.h>

/**@{*/

/* Translate register definitions so we can use pwr_common_v1.c and
 * pwr_common_v2.c
 */
#define PWR_CR            PWR_CR1
#define PWR_CSR           PWR_CSR1

#define PWR_CR_VOS_RANGE1 PWR_CR1_VOS_SCALE_1
#define PWR_CR_VOS_RANGE2 PWR_CR1_VOS_SCALE_2
#define PWR_CR_VOS_RANGE3 PWR_CR1_VOS_SCALE_3
#define PWR_CR_VOS_MASK   PWR_CR1_VOS_MASK
#define PWR_CR_DBP        PWR_CR1_DBP
#define PWR_CR_PLS_MASK   PWR_CR1_PLS_MASK
#define PWR_CR_PVDE       PWR_CR1_PVDE
#define PWR_CR_CSBF       PWR_CR1_CSBF
#define PWR_CR_CWUF       PWR_CR2_CWUPF1
#define PWR_CR_PDDS       PWR_CR1_PDDS
#define PWR_CR_LPDS       PWR_CR1_LPDS
#define PWR_CSR_EWUP      PWR_CSR1_EIWUP
#define PWR_CSR_PVDO      PWR_CSR1_PVDO
#define PWR_CSR_SBF       PWR_CSR1_SBF
#define PWR_CSR_WUF       PWR_CSR1_WUIF

#define pwr_enable_wakeup_pin  pwr_enable_wakeup_pin1
#define pwr_disable_wakeup_pin pwr_disable_wakeup_pin1

#include "../common/pwr_common_v1.c"
#include "../common/pwr_common_v2.c"

#undef pwr_enable_wakeup_pin
#undef pwr_disable_wakeup_pin

/*---------------------------------------------------------------------------*/
/** @brief Enable Wakeup Pin.

The wakeup pin is used for waking the processor from standby mode.
@param[in] pin Wakeup pin number
*/

void pwr_enable_wakeup_pin(enum pwr_wakeup_pin pin)
{
	PWR_CSR1 |= 1 << (8 + pin);
}

/*---------------------------------------------------------------------------*/
/** @brief Release Wakeup Pin.

The wakeup pin is used for general purpose I/O.
@param[in] pin Wakeup pin number
*/

void pwr_disable_wakeup_pin(enum pwr_wakeup_pin pin)
{
	PWR_CSR1 |= 1 << (8 + pin);
}


void pwr_enable_overdrive(void)
{
	PWR_CR1 |= PWR_CR1_ODEN;
	while (!(PWR_CSR1 & PWR_CSR1_ODRDY));
	PWR_CR1 |= PWR_CR1_ODSWEN;
	while (!(PWR_CSR1 & PWR_CSR1_ODSWRDY));
}

void pwr_disable_overdrive(void)
{
	PWR_CR1 &= ~(PWR_CR1_ODEN | PWR_CR1_ODSWEN);
	while (!(PWR_CSR1 & PWR_CSR1_ODSWRDY));
}

/**@}*/
