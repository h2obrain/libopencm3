/**
 * @defgroup jpeg_defines JPEG Defines
 * @brief <b>Defined Constants and Types for the STM32F7xx JPEG codec</b>
 * @ingroup STM32F7xx_defines
 * @version 1.0.0
 * @date 22 May 2019
 *
 * This library supports the JPEG codec in the STM32F7xx series of ARM Cortex
 * Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms @ref lgpl_license
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
/**@{*/

#ifndef LIBOPENCM3_STM32_F7_JPEG_H_
#define LIBOPENCM3_STM32_F7_JPEG_H_


/** @defgroup jpeg_registers JPEG Registers
 * @ingroup jpeg_defines
 * @brief JPEG Register Definitions
@{*/

/** JPEG codec control */
#define JPEG_CONFR0  (MMIO32(DSI_BASE + 0x0000))
/** JPEG codec configuration 1 */
#define JPEG_CONFR1  (MMIO32(DSI_BASE + 0x0004))
/** JPEG codec configuration 2 */
#define JPEG_CONFR2  (MMIO32(DSI_BASE + 0x0008))
/** JPEG codec configuration 3 */
#define JPEG_CONFR3  (MMIO32(DSI_BASE + 0x000C))
/** JPEG codec configuration 4 */
#define JPEG_CONFR4  (MMIO32(DSI_BASE + 0x0010))
/** JPEG codec configuration 5 */
#define JPEG_CONFR5  (MMIO32(DSI_BASE + 0x0014))
/** JPEG codec configuration 6 */
#define JPEG_CONFR6  (MMIO32(DSI_BASE + 0x0018))
/** JPEG codec configuration 7 */
#define JPEG_CONFR7  (MMIO32(DSI_BASE + 0x001C))
/** JPEG control */
#define JPEG_CR      (MMIO32(DSI_BASE + 0x0030))
/** JPEG status */
#define JPEG_SR      (MMIO32(DSI_BASE + 0x0034))
/** JPEG clear flag */
#define JPEG_CFR     (MMIO32(DSI_BASE + 0x0038))
/** JPEG data input */
#define JPEG_DIR     (MMIO32(DSI_BASE + 0x0040))
/** JPEG data output */
#define JPEG_DOR     (MMIO32(DSI_BASE + 0x0044))

/**@}*/



/*
 * JPEG flags/fields
 */


/** @defgroup jpeg_confr0_values JPEG_CONFR0 values
 * @ingroup jpeg_registers
 * @brief JPEG codec control
name  | bits | description | default
----- | ---- | ----------- | -------
START | 0    | Start       | 0
@{*/
typedef enum {
	JPEG_CONFR0_START                = 1<<0
} jpeg_confr0_flags_t;
#define JPEG_CONFR0_FLAGS_MASK         0x00000001
/**@}*/

/** @defgroup jpeg_confr1_values JPEG_CONFR1 values
 * @ingroup jpeg_registers
 * @brief JPEG codec configuration 1
name | bits | description                         | default
---- | ---- | ----------------------------------- | -------
HDR  | 8    | Header processing                   | 0
DE   | 3    | Codec operation as coder or decoder | 0
@{*/
typedef enum {
	JPEG_CONFR1_HDR                  = 1<<8,
	JPEG_CONFR1_DE                   = 1<<3
} jpeg_confr1_flags_t;
#define JPEG_CONFR1_FLAGS_MASK         0x00000108
/**@}*/

/** @defgroup jpeg_confr4567_values JPEG_CONFR4567 values
 * @ingroup jpeg_registers
 * @brief JPEG codec configuration 4-7
name | bits | description | default
---- | ---- | ----------- | -------
HA   | 1    | Huffman AC  | 0
HD   | 0    | Huffman DC  | 0
@{*/
typedef enum {
	JPEG_CONFR4567_HA                = 1<<1,
	JPEG_CONFR4567_HD                = 1<<0
} jpeg_confr4567_flags_t;
#define JPEG_CONFR4567_FLAGS_MASK      0x00000003
/**@}*/

/** @defgroup jpeg_cr_values JPEG_CR values
 * @ingroup jpeg_registers
 * @brief JPEG control
name   | bits | description                            | default
------ | ---- | -------------------------------------- | -------
OFF    | 14   | Output FIFO flush                      | 0
IFF    | 13   | Input FIFO flush                       | 0
ODMAEN | 12   | Output DMA enable                      | 0
IDMAEN | 11   | Input DMA enable                       | 0
HPDIE  | 6    | Header parsing done interrupt enable   | 0
EOCIE  | 5    | End of conversion interrupt enable     | 0
OFNEIE | 4    | Output FIFO not empty interrupt enable | 0
OFTIE  | 3    | Output FIFO threshold interrupt enable | 0
IFNFIE | 2    | Input FIFO not full interrupt enable   | 0
IFTIE  | 1    | Input FIFO threshold interrupt enable  | 0
JCEN   | 0    | JPEG core enable                       | 0
@{*/
typedef enum {
	JPEG_CR_OFF                      = 1<<14,
	JPEG_CR_IFF                      = 1<<13,
	JPEG_CR_ODMAEN                   = 1<<12,
	JPEG_CR_IDMAEN                   = 1<<11,
	JPEG_CR_HPDIE                    = 1<<6,
	JPEG_CR_EOCIE                    = 1<<5,
	JPEG_CR_OFNEIE                   = 1<<4,
	JPEG_CR_OFTIE                    = 1<<3,
	JPEG_CR_IFNFIE                   = 1<<2,
	JPEG_CR_IFTIE                    = 1<<1,
	JPEG_CR_JCEN                     = 1<<0
} jpeg_cr_flags_t;
#define JPEG_CR_FLAGS_MASK             0x0000787F
/**@}*/

/** @defgroup jpeg_sr_values JPEG_SR values
 * @ingroup jpeg_registers
 * @brief JPEG status
name  | bits | description                | default
----- | ---- | -------------------------- | -------
COF   | 7    | Codec operation flag       | 0
HPDF  | 6    | Header parsing done flag   | 0
EOCF  | 5    | End of conversion flag     | 0
OFNEF | 4    | Output FIFO not empty flag | 0
OFTF  | 4    | Output FIFO threshold flag | 0
IFNFF | 2    | Input FIFO not full flag   | 1
IFTF  | 1    | Input FIFO threshold flag  | 1
@{*/
typedef enum {
	JPEG_SR_COF                      = 1<<7,
	JPEG_SR_HPDF                     = 1<<6,
	JPEG_SR_EOCF                     = 1<<5,
	JPEG_SR_OFNEF                    = 1<<4,
	JPEG_SR_OFTF                     = 1<<4,
	JPEG_SR_IFNFF                    = 1<<2,
	JPEG_SR_IFTF                     = 1<<1
} jpeg_sr_flags_t;
#define JPEG_SR_FLAGS_MASK             0x000000F6
/**@}*/

/** @defgroup jpeg_cfr_values JPEG_CFR values
 * @ingroup jpeg_registers
 * @brief JPEG clear flag
name  | bits | description                    | default
----- | ---- | ------------------------------ | -------
CHPDF | 6    | Clear header parsing done flag | 0
CEOCF | 5    | Clear end of conversion flag   | 0
@{*/
typedef enum {
	JPEG_CFR_CHPDF                   = 1<<6,
	JPEG_CFR_CEOCF                   = 1<<5
} jpeg_cfr_flags_t;
#define JPEG_CFR_FLAGS_MASK            0x00000060
/**@}*/

#endif /* LIBOPENCM3_STM32_F7_JPEG_H_ */

/**@}*/
