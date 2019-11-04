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


/* --- JPEG registers --- */
/** @defgroup jpeg_registers JPEG Registers
 * @ingroup jpeg_defines
 * @brief JPEG Register Definitions
@{*/

/** JPEG codec control */
#define JPEG_CONFR0          (MMIO32(JPEG_BASE + 0x0000))
/** JPEG codec configuration 1 */
#define JPEG_CONFR1          (MMIO32(JPEG_BASE + 0x0004))
/** JPEG codec configuration 2 */
#define JPEG_CONFR2          (MMIO32(JPEG_BASE + 0x0008))
/** JPEG codec configuration 3 */
#define JPEG_CONFR3          (MMIO32(JPEG_BASE + 0x000C))
/** JPEG codec configuration 4 */
#define JPEG_CONFR4          (MMIO32(JPEG_BASE + 0x0010))
/** JPEG codec configuration 5 */
#define JPEG_CONFR5          (MMIO32(JPEG_BASE + 0x0014))
/** JPEG codec configuration 6 */
#define JPEG_CONFR6          (MMIO32(JPEG_BASE + 0x0018))
/** JPEG codec configuration 7 */
#define JPEG_CONFR7          (MMIO32(JPEG_BASE + 0x001C))
/** JPEG control */
#define JPEG_CR              (MMIO32(JPEG_BASE + 0x0030))
/** JPEG status */
#define JPEG_SR              (MMIO32(JPEG_BASE + 0x0034))
/** JPEG clear flag */
#define JPEG_CFR             (MMIO32(JPEG_BASE + 0x0038))
/** JPEG data input */
#define JPEG_DIR             (MMIO32(JPEG_BASE + 0x0040))
/** JPEG data output */
#define JPEG_DOR             (MMIO32(JPEG_BASE + 0x0044))

/**@}*/






/* --- JPEG register bits --- */

/** @defgroup jpeg_confr0_values JPEG_CONFR0 values
 * @ingroup jpeg_registers
 * @brief JPEG codec control
name     | bits | description | default
-------- | ---- | ----------- | -------
START    | 0    | Start       | 0
Reserved | 1:31 |             | 0
@{*/
typedef enum {
	JPEG_CONFR0_START                = 1<<0
} jpeg_confr0_flags_t;
#define JPEG_CONFR0_FLAGS_MASK         0x00000001
/**@}*/

/** @defgroup jpeg_confr1_values JPEG_CONFR1 values
 * @ingroup jpeg_registers
 * @brief JPEG codec configuration 1
name       | bits  | description                         | default
---------- | ----- | ----------------------------------- | -------
NF         | 0:1   | Number of color components          | 0
Reserved   | 2     |                                     | 0
DE         | 3     | Codec operation as coder or decoder | 0
COLORSPACE | 4:5   | Color space                         | 0
NS         | 6:7   | Number of components for scan       | 0
HDR        | 8     | Header processing                   | 0
Reserved   | 9:15  |                                     | 0
YSIZE      | 16:31 | Y Size                              | 0
@{*/
typedef enum {
	JPEG_CONFR1_DE                   = 1<<3,
	JPEG_CONFR1_HDR                  = 1<<8
} jpeg_confr1_flags_t;
#define JPEG_CONFR1_FLAGS_MASK         0x00000108
#define JPEG_CONFR1_NF_MASK            0x00000003
#define JPEG_CONFR1_NF_SHIFT           0
#define JPEG_CONFR1_COLORSPACE_MASK    0x00000003
#define JPEG_CONFR1_COLORSPACE_SHIFT   4
#define JPEG_CONFR1_NS_MASK            0x00000003
#define JPEG_CONFR1_NS_SHIFT           6
#define JPEG_CONFR1_YSIZE_MASK         0x0000FFFF
#define JPEG_CONFR1_YSIZE_SHIFT        16

typedef enum {
	JPEG_COLORSPACE_GRAYSCALE,
	JPEG_COLORSPACE_YUV,
	JPEG_COLORSPACE_RGB,
	JPEG_COLORSPACE_CMYK
} jpeg_confr1_colorspace_t;
/**@}*/

/** @defgroup jpeg_confr2_values JPEG_CONFR2 values
 * @ingroup jpeg_registers
 * @brief JPEG codec configuration 2
name     | bits  | description    | default
-------- | ----- | -------------- | -------
NMCU     | 0:25  | Number of MCUs | 0
Reserved | 26:31 |                | 0
@{*/
#define JPEG_CONFR2_NMCU_MASK          0x03FFFFFF
#define JPEG_CONFR2_NMCU_SHIFT         0
/**@}*/

/** @defgroup jpeg_confr3_values JPEG_CONFR3 values
 * @ingroup jpeg_registers
 * @brief JPEG codec configuration 3
name     | bits  | description | default
-------- | ----- | ----------- | -------
Reserved | 0:15  |             | 0
XSIZE    | 16:31 | X size      | 0
@{*/
#define JPEG_CONFR3_XSIZE_MASK         0x0000FFFF
#define JPEG_CONFR3_XSIZE_SHIFT        16
/**@}*/

/** @defgroup jpeg_confr4567_values JPEG_CONFR4567 values
 * @ingroup jpeg_registers
 * @brief JPEG codec configuration 4-7
name     | bits  | description                | default
-------- | ----- | -------------------------- | -------
HD       | 0     | Huffman DC                 | 0
HA       | 1     | Huffman AC                 | 0
QT       | 2:3   | Quantization table         | 0
NB       | 4:7   | Number of blocks           | 0
VSF      | 8:11  | Vertical sampling factor   | 0
HSF      | 12:15 | Horizontal sampling factor | 0
Reserved | 16:31 |                            | 0
@{*/
typedef enum {
	JPEG_CONFR4567_HD                 = 1<<0,
	JPEG_CONFR4567_HA                 = 1<<1
} jpeg_confr4567_flags_t;
#define JPEG_CONFR4567_FLAGS_MASK       0x00000003
#define JPEG_CONFR4567_QT_MASK          0x00000003
#define JPEG_CONFR4567_QT_SHIFT         2
#define JPEG_CONFR4567_NB_MASK          0x0000000F
#define JPEG_CONFR4567_NB_SHIFT         4
#define JPEG_CONFR4567_VSF_MASK         0x0000000F
#define JPEG_CONFR4567_VSF_SHIFT        8
#define JPEG_CONFR4567_HSF_MASK         0x0000000F
#define JPEG_CONFR4567_HSF_SHIFT        12
/**@}*/

/** @defgroup jpeg_cr_values JPEG_CR values
 * @ingroup jpeg_registers
 * @brief JPEG control
name     | bits  | description                            | default
-------- | ----- | -------------------------------------- | -------
JCEN     | 0     | JPEG core enable                       | 0
IFTIE    | 1     | Input FIFO threshold interrupt enable  | 0
IFNFIE   | 2     | Input FIFO not full interrupt enable   | 0
OFTIE    | 3     | Output FIFO threshold interrupt enable | 0
OFNEIE   | 4     | Output FIFO not empty interrupt enable | 0
EOCIE    | 5     | End of conversion interrupt enable     | 0
HPDIE    | 6     | Header parsing done interrupt enable   | 0
Reserved | 7:10  |                                        | 0
IDMAEN   | 11    | Input DMA enable                       | 0
ODMAEN   | 12    | Output DMA enable                      | 0
IFF      | 13    | Input FIFO flush                       | 0
OFF      | 14    | Output FIFO flush                      | 0
Reserved | 15:31 |                                        | 0
@{*/
typedef enum {
	JPEG_CR_JCEN                     = 1<<0,
	JPEG_CR_IFTIE                    = 1<<1,
	JPEG_CR_IFNFIE                   = 1<<2,
	JPEG_CR_OFTIE                    = 1<<3,
	JPEG_CR_OFNEIE                   = 1<<4,
	JPEG_CR_EOCIE                    = 1<<5,
	JPEG_CR_HPDIE                    = 1<<6,
	JPEG_CR_IDMAEN                   = 1<<11,
	JPEG_CR_ODMAEN                   = 1<<12,
	JPEG_CR_IFF                      = 1<<13,
	JPEG_CR_OFF                      = 1<<14
} jpeg_cr_flags_t;
#define JPEG_CR_FLAGS_MASK             0x0000787F
/**@}*/

/** @defgroup jpeg_sr_values JPEG_SR values
 * @ingroup jpeg_registers
 * @brief JPEG status
name     | bits | description                | default
-------- | ---- | -------------------------- | -------
Reserved | 0    |                            | 0
IFTF     | 1    | Input FIFO threshold flag  | 1
IFNFF    | 2    | Input FIFO not full flag   | 1
OFTF     | 3    | Output FIFO threshold flag | 0
OFNEF    | 4    | Output FIFO not empty flag | 0
EOCF     | 5    | End of conversion flag     | 0
HPDF     | 6    | Header parsing done flag   | 0
COF      | 7    | Codec operation flag       | 0
Reserved | 8:31 |                            | 0
@{*/
typedef enum {
	JPEG_SR_IFTF                     = 1<<1,
	JPEG_SR_IFNFF                    = 1<<2,
	JPEG_SR_OFTF                     = 1<<3,
	JPEG_SR_OFNEF                    = 1<<4,
	JPEG_SR_EOCF                     = 1<<5,
	JPEG_SR_HPDF                     = 1<<6,
	JPEG_SR_COF                      = 1<<7
} jpeg_sr_flags_t;
#define JPEG_SR_FLAGS_MASK             0x000000FE
/**@}*/

/** @defgroup jpeg_cfr_values JPEG_CFR values
 * @ingroup jpeg_registers
 * @brief JPEG clear flag
name     | bits | description                    | default
-------- | ---- | ------------------------------ | -------
Reserved | 0:4  |                                | 0
CEOCF    | 5    | Clear end of conversion flag   | 0
CHPDF    | 6    | Clear header parsing done flag | 0
Reserved | 7:31 |                                | 0
@{*/
typedef enum {
	JPEG_CFR_CEOCF                   = 1<<5,
	JPEG_CFR_CHPDF                   = 1<<6
} jpeg_cfr_flags_t;
#define JPEG_CFR_FLAGS_MASK            0x00000060
/**@}*/

/** @defgroup jpeg_dir_values JPEG_DIR values
 * @ingroup jpeg_registers
 * @brief JPEG data input
name   | bits | description     | default
------ | ---- | --------------- | -------
DATAIN | 0:31 | Data input FIFO | 0
@{*/
#define JPEG_DIR_DATAIN_MASK           0xFFFFFFFF
#define JPEG_DIR_DATAIN_SHIFT          0
/**@}*/

/** @defgroup jpeg_dor_values JPEG_DOR values
 * @ingroup jpeg_registers
 * @brief JPEG data output
name    | bits | description      | default
------- | ---- | ---------------- | -------
DATAOUT | 0:31 | Data output FIFO | 0
@{*/
#define JPEG_DOR_DATAOUT_MASK          0xFFFFFFFF
#define JPEG_DOR_DATAOUT_SHIFT         0
/**@}*/


#endif /* LIBOPENCM3_STM32_F7_JPEG_H_ */

/**@}*/

