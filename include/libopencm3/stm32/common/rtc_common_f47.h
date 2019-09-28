/** @addtogroup rtc_defines

@author @htmlonly &copy; @endhtmlonly 2012 Karl Palsson <karlp@tweak.net.au>

*/
/*
 * This file is part of the libopencm3 project.
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

/* THIS FILE SHOULD NOT BE INCLUDED DIRECTLY, BUT ONLY VIA RTC.H
The order of header inclusion is important. rtc.h includes the device
specific memorymap.h header before including this header file.*/

/** @cond */
#ifdef LIBOPENCM3_RTC_H
/** @endcond */
#ifndef LIBOPENCM3_RTC3_H
#define LIBOPENCM3_RTC3_H

#include <libopencm3/stm32/common/rtc_common_l1f024.h>

BEGIN_DECLS

void rtc_enable_wakeup_timer(void);
void rtc_disable_wakeup_timer(void);
void rtc_enable_wakeup_timer_interrupt(void);
void rtc_disable_wakeup_timer_interrupt(void);

END_DECLS
/**@}*/

#endif  /* RTC3_H */
/** @cond */
#else
#warning "rtc_common_f47.h should not be included explicitly, only via rtc.h"
#endif
/** @endcond */


