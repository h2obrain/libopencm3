/** @addtogroup dsi_defines
 *
 * @version 1.0.0
 *
 * @date 7 July 2016
 *
 * @author @htmlonly &copy; @endhtmlonly 2016
 * Chuck McManis <cmcmanis@mcmanis.com>
 * 
 * This library supports the Display Serial Interface Host and Wrapper in
 * the STM32F4xx and STM32F7xx series of ARM Cortex Microcontrollers by
 * ST Microelectronics.
 * 
 * LGPL License Terms @ref lgpl_license
 */

/*
 * STM32F4/7 DSI Host Defines
 *
 * Copyright (C) 2016, Chuck McManis <cmcmanis@mcmanis.com>
 *
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
 *
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/memorymap.h>

/** @cond */
#ifndef DSI_H
/** @endcond */
#define DSI_H



/* --- DSI Host registers --- */
/** @defgroup dsi_host_registers DSI Host Registers
 * @ingroup dsi_defines
 * @brief DSI Host Control Registers
@{*/

/** DSI Host version */
#define DSI_VR               (MMIO32(DSI_BASE + 0x0000))
/** DSI Host control */
#define DSI_CR               (MMIO32(DSI_BASE + 0x0004))
/** DSI Host clock control */
#define DSI_CCR              (MMIO32(DSI_BASE + 0x0008))
/** DSI Host LTDC VCID */
#define DSI_LVCIDR           (MMIO32(DSI_BASE + 0x000C))
/** DSI Host LTDC color coding */
#define DSI_LCOLCR           (MMIO32(DSI_BASE + 0x0010))
/** DSI Host LTDC polarity configuration */
#define DSI_LPCR             (MMIO32(DSI_BASE + 0x0014))
/** DSI Host low-power mode configuration */
#define DSI_LPMCR            (MMIO32(DSI_BASE + 0x0018))
/** DSI Host protocol configuration */
#define DSI_PCR              (MMIO32(DSI_BASE + 0x002C))
/** DSI Host generic VCID */
#define DSI_GVCIDR           (MMIO32(DSI_BASE + 0x0030))
/** DSI Host mode configuration */
#define DSI_MCR              (MMIO32(DSI_BASE + 0x0034))
/** DSI Host video mode configuration */
#define DSI_VMCR             (MMIO32(DSI_BASE + 0x0038))
/** DSI Host video packet configuration */
#define DSI_VPCR             (MMIO32(DSI_BASE + 0x003C))
/** DSI Host video chunks configuration */
#define DSI_VCCR             (MMIO32(DSI_BASE + 0x0040))
/** DSI Host video null packet configuration */
#define DSI_VNPCR            (MMIO32(DSI_BASE + 0x0044))
/** DSI Host video HSA configuration */
#define DSI_VHSACR           (MMIO32(DSI_BASE + 0x0048))
/** DSI Host video HBP configuration */
#define DSI_VHBPCR           (MMIO32(DSI_BASE + 0x004C))
/** DSI Host video line configuration */
#define DSI_VLCR             (MMIO32(DSI_BASE + 0x0050))
/** DSI Host video VSA configuration */
#define DSI_VVSACR           (MMIO32(DSI_BASE + 0x0054))
/** DSI Host video VBP configuration */
#define DSI_VVBPCR           (MMIO32(DSI_BASE + 0x0058))
/** DSI Host video VFP configuration */
#define DSI_VVFPCR           (MMIO32(DSI_BASE + 0x005C))
/** DSI Host video VA configuration */
#define DSI_VVACR            (MMIO32(DSI_BASE + 0x0060))
/** DSI Host LTDC command configuration */
#define DSI_LCCR             (MMIO32(DSI_BASE + 0x0064))
/** DSI Host command mode configuration */
#define DSI_CMCR             (MMIO32(DSI_BASE + 0x0068))
/** DSI Host generic header configuration */
#define DSI_GHCR             (MMIO32(DSI_BASE + 0x006C))
/** DSI Host generic payload data */
#define DSI_GPDR             (MMIO32(DSI_BASE + 0x0070))
/** DSI Host generic packet status */
#define DSI_GPSR             (MMIO32(DSI_BASE + 0x0074))
/** DSI Host timeout counter configuration 0 */
#define DSI_TCCR0            (MMIO32(DSI_BASE + 0x0078))
/** DSI Host timeout counter configuration 1 */
#define DSI_TCCR1            (MMIO32(DSI_BASE + 0x007C))
/** DSI Host timeout counter configuration 2 */
#define DSI_TCCR2            (MMIO32(DSI_BASE + 0x0080))
/** DSI Host timeout counter configuration 3 */
#define DSI_TCCR3            (MMIO32(DSI_BASE + 0x0084))
/** DSI Host timeout counter configuration 4 */
#define DSI_TCCR4            (MMIO32(DSI_BASE + 0x0088))
/** DSI Host timeout counter configuration 5 */
#define DSI_TCCR5            (MMIO32(DSI_BASE + 0x008C))
/** DSI Host clock lane configuration */
#define DSI_CLCR             (MMIO32(DSI_BASE + 0x0094))
/** DSI Host clock lane timer configuration */
#define DSI_CLTCR            (MMIO32(DSI_BASE + 0x0098))
/** DSI Host data lane timer configuration */
#define DSI_DLTCR            (MMIO32(DSI_BASE + 0x009C))
/** DSI Host PHY control */
#define DSI_PCTLR            (MMIO32(DSI_BASE + 0x00A0))
/** DSI Host PHY configuration */
#define DSI_PCONFR           (MMIO32(DSI_BASE + 0x00A4))
/** DSI Host PHY ULPS control */
#define DSI_PUCR             (MMIO32(DSI_BASE + 0x00A8))
/** DSI Host PHY TX triggers configuration */
#define DSI_PTTCR            (MMIO32(DSI_BASE + 0x00AC))
/** DSI Host PHY status */
#define DSI_PSR              (MMIO32(DSI_BASE + 0x00B0))
/** DSI Host interrupt and status 0 */
#define DSI_ISR0             (MMIO32(DSI_BASE + 0x00BC))
/** DSI Host interrupt and status 1 */
#define DSI_ISR1             (MMIO32(DSI_BASE + 0x00C0))
/** DSI Host interrupt enable 0 */
#define DSI_IER0             (MMIO32(DSI_BASE + 0x00C4))
/** DSI Host interrupt enable 1 */
#define DSI_IER1             (MMIO32(DSI_BASE + 0x00C8))
/** DSI Host force interrupt 0 */
#define DSI_FIR0             (MMIO32(DSI_BASE + 0x00D8))
/** DSI Host force interrupt 1 */
#define DSI_FIR1             (MMIO32(DSI_BASE + 0x00DC))
/** DSI Host video shadow control */
#define DSI_VSCR             (MMIO32(DSI_BASE + 0x0100))
/** DSI Host LTDC current VCID */
#define DSI_LCVCIDR          (MMIO32(DSI_BASE + 0x010C))
/** DSI Host LTDC current color coding */
#define DSI_LCCCR            (MMIO32(DSI_BASE + 0x0110))
/** DSI Host low-power mode current configuration */
#define DSI_LPMCCR           (MMIO32(DSI_BASE + 0x0118))
/** DSI Host video mode current configuration */
#define DSI_VMCCR            (MMIO32(DSI_BASE + 0x0138))
/** DSI Host video packet current configuration */
#define DSI_VPCCR            (MMIO32(DSI_BASE + 0x013C))
/** DSI Host video chunks current configuration */
#define DSI_VCCCR            (MMIO32(DSI_BASE + 0x0140))
/** DSI Host video null packet current configuration */
#define DSI_VNPCCR           (MMIO32(DSI_BASE + 0x0144))
/** DSI Host video HSA current configuration */
#define DSI_VHSACCR          (MMIO32(DSI_BASE + 0x0148))
/** DSI Host video HBP current configuration */
#define DSI_VHBPCCR          (MMIO32(DSI_BASE + 0x014C))
/** DSI Host video line current configuration */
#define DSI_VLCCR            (MMIO32(DSI_BASE + 0x0150))
/** DSI Host video VSA current configuration */
#define DSI_VVSACCR          (MMIO32(DSI_BASE + 0x0154))
/** DSI Host video VBP current configuration */
#define DSI_VVBPCCR          (MMIO32(DSI_BASE + 0x0158))
/** DSI Host video VFP current configuration */
#define DSI_VVFPCCR          (MMIO32(DSI_BASE + 0x015C))
/** DSI Host video VA current configuration */
#define DSI_VVACCR           (MMIO32(DSI_BASE + 0x0160))

/**@}*/

/* --- DSI Wrapper registers --- */
/** @defgroup dsi_wrapper_registers DSI Wrapper Registers
 * @ingroup dsi_defines
 * @brief DSI Wrapper Control Registers
@{*/

/** DSI wrapper configuration */
#define DSI_WCFGR            (MMIO32(DSI_BASE + 0x0400))
/** DSI wrapper control */
#define DSI_WCR              (MMIO32(DSI_BASE + 0x0404))
/** DSI wrapper interrupt enable */
#define DSI_WIER             (MMIO32(DSI_BASE + 0x0408))
/** DSI wrapper interrupt and status */
#define DSI_WISR             (MMIO32(DSI_BASE + 0x040C))
/** DSI wrapper interrupt flag clear */
#define DSI_WIFCR            (MMIO32(DSI_BASE + 0x0410))
/** DSI wrapper PHY configuration 0 */
#define DSI_WPCR0            (MMIO32(DSI_BASE + 0x0418))
/** DSI wrapper PHY configuration 1 */
#define DSI_WPCR1            (MMIO32(DSI_BASE + 0x041C))
/** DSI wrapper PHY configuration 2 */
#define DSI_WPCR2            (MMIO32(DSI_BASE + 0x0420))
/** DSI wrapper PHY configuration 3 */
#define DSI_WPCR3            (MMIO32(DSI_BASE + 0x0424))
/** DSI wrapper PHY configuration 4 */
#define DSI_WPCR4            (MMIO32(DSI_BASE + 0x0428))
/** DSI wrapper regulator and PLL control */
#define DSI_WRPCR            (MMIO32(DSI_BASE + 0x0430))

/**@}*/





/* --- DSI_HOST register bits --- */

/** @defgroup dsi_vr_values DSI_VR values
 * @ingroup dsi_host_registers
 * @brief DSI Host version
name    | bits | description             | default
------- | ---- | ----------------------- | -------
VERSION | 0:31 | Version of the DSI Host | 825438250
@{*/
#define DSI_VR_VERSION_MASK            0xFFFFFFFF
#define DSI_VR_VERSION_SHIFT           0
/**@}*/

/** @defgroup dsi_cr_values DSI_CR values
 * @ingroup dsi_host_registers
 * @brief DSI Host control
name     | bits | description | default
-------- | ---- | ----------- | -------
EN       | 0    | Enable      | 0
Reserved | 1:31 |             | 0
@{*/
typedef enum {
	DSI_CR_EN                        = 1<<0
} dsi_cr_flags_t;
#define DSI_CR_FLAGS_MASK              0x00000001
/**@}*/

/** @defgroup dsi_ccr_values DSI_CCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host clock control
name     | bits  | description              | default
-------- | ----- | ------------------------ | -------
TXECKDIV | 0:7   | TX escape clock division | 0
TOCKDIV  | 8:15  | Timeout clock division   | 0
Reserved | 16:31 |                          | 0
@{*/
#define DSI_CCR_TXECKDIV_MASK          0x000000FF
#define DSI_CCR_TXECKDIV_SHIFT         0
#define DSI_CCR_TOCKDIV_MASK           0x000000FF
#define DSI_CCR_TOCKDIV_SHIFT          8
/**@}*/

/** @defgroup dsi_lvcidr_values DSI_LVCIDR values
 * @ingroup dsi_host_registers
 * @brief DSI Host LTDC VCID
name     | bits | description        | default
-------- | ---- | ------------------ | -------
VCID     | 0:1  | Virtual channel ID | 0
Reserved | 2:31 |                    | 0
@{*/
#define DSI_LVCIDR_VCID_MASK           0x00000003
#define DSI_LVCIDR_VCID_SHIFT          0
/**@}*/

/** @defgroup dsi_lcolcr_values DSI_LCOLCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host LTDC color coding
name     | bits | description           | default
-------- | ---- | --------------------- | -------
COLC     | 0:3  | Color coding          | 0
Reserved | 4:7  |                       | 0
LPE      | 8    | Loosely packet enable | 0
Reserved | 9:31 |                       | 0
@{*/
typedef enum {
	DSI_LCOLCR_LPE                   = 1<<8
} dsi_lcolcr_flags_t;
#define DSI_LCOLCR_FLAGS_MASK          0x00000100
#define DSI_LCOLCR_COLC_MASK           0x0000000F
#define DSI_LCOLCR_COLC_SHIFT          0
/**@}*/

/** @defgroup dsi_lpcr_values DSI_LPCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host LTDC polarity configuration
name     | bits | description          | default
-------- | ---- | -------------------- | -------
DEP      | 0    | Data enable polarity | 0
VSP      | 1    | VSYNC polarity       | 0
HSP      | 2    | HSYNC polarity       | 0
Reserved | 3:31 |                      | 0
@{*/
typedef enum {
	DSI_LPCR_DEP                     = 1<<0,
	DSI_LPCR_VSP                     = 1<<1,
	DSI_LPCR_HSP                     = 1<<2
} dsi_lpcr_flags_t;
#define DSI_LPCR_FLAGS_MASK            0x00000007
/**@}*/

/** @defgroup dsi_lpmcr_values DSI_LPMCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host low-power mode configuration
name     | bits  | description              | default
-------- | ----- | ------------------------ | -------
VLPSIZE  | 0:7   | VACT largest packet size | 0
Reserved | 8:15  |                          | 0
LPSIZE   | 16:23 | Largest packet size      | 0
Reserved | 24:31 |                          | 0
@{*/
#define DSI_LPMCR_VLPSIZE_MASK         0x000000FF
#define DSI_LPMCR_VLPSIZE_SHIFT        0
#define DSI_LPMCR_LPSIZE_MASK          0x000000FF
#define DSI_LPMCR_LPSIZE_SHIFT         16
/**@}*/

/** @defgroup dsi_pcr_values DSI_PCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host protocol configuration
name     | bits | description              | default
-------- | ---- | ------------------------ | -------
ETTXE    | 0    | EoTp transmission enable | 0
ETRXE    | 1    | EoTp reception enable    | 0
BTAE     | 2    | Bus-turn-around enable   | 0
ECCRXE   | 3    | ECC reception enable     | 0
CRCRXE   | 4    | CRC reception enable     | 0
Reserved | 5:31 |                          | 0
@{*/
typedef enum {
	DSI_PCR_ETTXE                    = 1<<0,
	DSI_PCR_ETRXE                    = 1<<1,
	DSI_PCR_BTAE                     = 1<<2,
	DSI_PCR_ECCRXE                   = 1<<3,
	DSI_PCR_CRCRXE                   = 1<<4
} dsi_pcr_flags_t;
#define DSI_PCR_FLAGS_MASK             0x0000001F
/**@}*/

/** @defgroup dsi_gvcidr_values DSI_GVCIDR values
 * @ingroup dsi_host_registers
 * @brief DSI Host generic VCID
name     | bits | description        | default
-------- | ---- | ------------------ | -------
VCID     | 0:1  | Virtual channel ID | 0
Reserved | 2:31 |                    | 0
@{*/
#define DSI_GVCIDR_VCID_MASK           0x00000003
#define DSI_GVCIDR_VCID_SHIFT          0
/**@}*/

/** @defgroup dsi_mcr_values DSI_MCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host mode configuration
name     | bits | description  | default
-------- | ---- | ------------ | -------
CMDM     | 0    | Command mode | 1
Reserved | 1:31 |              | 0
@{*/
typedef enum {
	DSI_MCR_CMDM                     = 1<<0
} dsi_mcr_flags_t;
#define DSI_MCR_FLAGS_MASK             0x00000001
/**@}*/

/** @defgroup dsi_vmcr_values DSI_VMCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video mode configuration
name     | bits  | description                              | default
-------- | ----- | ---------------------------------------- | -------
VMT      | 0:1   | Video mode type                          | 0
Reserved | 2:7   |                                          | 0
LPVSAE   | 8     | Low-power vertical sync active enable    | 0
LPVBPE   | 9     | Low-power vertical back-porch enable     | 0
LPVFPE   | 10    | Low-power vertical front-porch enable    | 0
LPVAE    | 11    | Low-power vertical active enable         | 0
LPHBPE   | 12    | Low-power horizontal back-porch enable   | 0
LPHFPE   | 13    | Low-power horizontal front-porch enable  | 0
FBTAAE   | 14    | Frame bus-turn-around acknowledge enable | 0
LPCE     | 15    | Low-power command enable                 | 0
PGE      | 16    | Pattern generator enable                 | 0
Reserved | 17:19 |                                          | 0
PGM      | 20    | Pattern generator mode                   | 0
Reserved | 21:23 |                                          | 0
PGO      | 24    | Pattern generator orientation            | 0
Reserved | 25:31 |                                          | 0
@{*/
typedef enum {
	DSI_VMCR_LPVSAE                  = 1<<8,
	DSI_VMCR_LPVBPE                  = 1<<9,
	DSI_VMCR_LPVFPE                  = 1<<10,
	DSI_VMCR_LPVAE                   = 1<<11,
	DSI_VMCR_LPHBPE                  = 1<<12,
	DSI_VMCR_LPHFPE                  = 1<<13,
	DSI_VMCR_FBTAAE                  = 1<<14,
	DSI_VMCR_LPCE                    = 1<<15,
	DSI_VMCR_PGE                     = 1<<16,
	DSI_VMCR_PGM                     = 1<<20,
	DSI_VMCR_PGO                     = 1<<24
} dsi_vmcr_flags_t;
#define DSI_VMCR_FLAGS_MASK            0x0111FF00
#define DSI_VMCR_VMT_MASK              0x00000003
#define DSI_VMCR_VMT_SHIFT             0
typedef enum {
	DSI_VMCR_VMT_NON_BURST_PULSE = 0,
	DSI_VMCR_VMT_NON_BURST_EVENT = 1,
	DSI_VMCR_VMT_BURST           = 2,
	DSI_VMCR_VMT_BURST_          = 3
} dsi_vmcr_vmt_t;
/**@}*/

/** @defgroup dsi_vpcr_values DSI_VPCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video packet configuration
name     | bits  | description       | default
-------- | ----- | ----------------- | -------
VPSIZE   | 0:13  | Video packet size | 0
Reserved | 14:31 |                   | 0
@{*/
#define DSI_VPCR_VPSIZE_MASK           0x00003FFF
#define DSI_VPCR_VPSIZE_SHIFT          0
/**@}*/

/** @defgroup dsi_vccr_values DSI_VCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video chunks configuration
name     | bits  | description      | default
-------- | ----- | ---------------- | -------
NUMC     | 0:12  | Number of chunks | 0
Reserved | 13:31 |                  | 0
@{*/
#define DSI_VCCR_NUMC_MASK             0x00001FFF
#define DSI_VCCR_NUMC_SHIFT            0
/**@}*/

/** @defgroup dsi_vnpcr_values DSI_VNPCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video null packet configuration
name     | bits  | description      | default
-------- | ----- | ---------------- | -------
NPSIZE   | 0:12  | Null packet size | 0
Reserved | 13:31 |                  | 0
@{*/
#define DSI_VNPCR_NPSIZE_MASK          0x00001FFF
#define DSI_VNPCR_NPSIZE_SHIFT         0
/**@}*/

/** @defgroup dsi_vhsacr_values DSI_VHSACR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video HSA configuration
name     | bits  | description                            | default
-------- | ----- | -------------------------------------- | -------
HSA      | 0:11  | Horizontal synchronism active duration | 0
Reserved | 12:31 |                                        | 0
@{*/
#define DSI_VHSACR_HSA_MASK            0x00000FFF
#define DSI_VHSACR_HSA_SHIFT           0
/**@}*/

/** @defgroup dsi_vhbpcr_values DSI_VHBPCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video HBP configuration
name     | bits  | description                    | default
-------- | ----- | ------------------------------ | -------
HBP      | 0:11  | Horizontal back-porch duration | 0
Reserved | 12:31 |                                | 0
@{*/
#define DSI_VHBPCR_HBP_MASK            0x00000FFF
#define DSI_VHBPCR_HBP_SHIFT           0
/**@}*/

/** @defgroup dsi_vlcr_values DSI_VLCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video line configuration
name     | bits  | description              | default
-------- | ----- | ------------------------ | -------
HLINE    | 0:14  | Horizontal line duration | 0
Reserved | 15:31 |                          | 0
@{*/
#define DSI_VLCR_HLINE_MASK            0x00007FFF
#define DSI_VLCR_HLINE_SHIFT           0
/**@}*/

/** @defgroup dsi_vvsacr_values DSI_VVSACR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video VSA configuration
name     | bits  | description                          | default
-------- | ----- | ------------------------------------ | -------
VSA      | 0:9   | Vertical synchronism active duration | 0
Reserved | 10:31 |                                      | 0
@{*/
#define DSI_VVSACR_VSA_MASK            0x000003FF
#define DSI_VVSACR_VSA_SHIFT           0
/**@}*/

/** @defgroup dsi_vvbpcr_values DSI_VVBPCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video VBP configuration
name     | bits  | description                  | default
-------- | ----- | ---------------------------- | -------
VBP      | 0:9   | Vertical back-porch duration | 0
Reserved | 10:31 |                              | 0
@{*/
#define DSI_VVBPCR_VBP_MASK            0x000003FF
#define DSI_VVBPCR_VBP_SHIFT           0
/**@}*/

/** @defgroup dsi_vvfpcr_values DSI_VVFPCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video VFP configuration
name     | bits  | description                   | default
-------- | ----- | ----------------------------- | -------
VFP      | 0:9   | Vertical front-porch duration | 0
Reserved | 10:31 |                               | 0
@{*/
#define DSI_VVFPCR_VFP_MASK            0x000003FF
#define DSI_VVFPCR_VFP_SHIFT           0
/**@}*/

/** @defgroup dsi_vvacr_values DSI_VVACR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video VA configuration
name     | bits  | description              | default
-------- | ----- | ------------------------ | -------
VA       | 0:13  | Vertical active duration | 0
Reserved | 14:31 |                          | 0
@{*/
#define DSI_VVACR_VA_MASK              0x00003FFF
#define DSI_VVACR_VA_SHIFT             0
/**@}*/

/** @defgroup dsi_lccr_values DSI_LCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host LTDC command configuration
name     | bits  | description  | default
-------- | ----- | ------------ | -------
CMDSIZE  | 0:15  | Command size | 0
Reserved | 16:31 |              | 0
@{*/
#define DSI_LCCR_CMDSIZE_MASK          0x0000FFFF
#define DSI_LCCR_CMDSIZE_SHIFT         0
/**@}*/

/** @defgroup dsi_cmcr_values DSI_CMCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host command mode configuration
name     | bits  | description                                      | default
-------- | ----- | ------------------------------------------------ | -------
TEARE    | 0     | Tearing effect acknowledge request enable        | 0
ARE      | 1     | Acknowledge request enable                       | 0
Reserved | 2:7   |                                                  | 0
GSW0TX   | 8     | Generic short write zero parameters transmission | 0
GSW1TX   | 9     | Generic short write one parameters transmission  | 0
GSW2TX   | 10    | Generic short write two parameters transmission  | 0
GSR0TX   | 11    | Generic short read zero parameters transmission  | 0
GSR1TX   | 12    | Generic short read one parameters transmission   | 0
GSR2TX   | 13    | Generic short read two parameters transmission   | 0
GLWTX    | 14    | Generic long write transmission                  | 0
Reserved | 15    |                                                  | 0
DSW0TX   | 16    | DCS short write zero parameter transmission      | 0
DSW1TX   | 17    | DCS short read one parameter transmission        | 0
DSR0TX   | 18    | DCS short read zero parameter transmission       | 0
DLWTX    | 19    | DCS long write transmission                      | 0
Reserved | 20:23 |                                                  | 0
MRDPS    | 24    | Maximum read packet size                         | 0
Reserved | 25:31 |                                                  | 0
@{*/
typedef enum {
	DSI_CMCR_TEARE                   = 1<<0,
	DSI_CMCR_ARE                     = 1<<1,
	DSI_CMCR_GSW0TX                  = 1<<8,
	DSI_CMCR_GSW1TX                  = 1<<9,
	DSI_CMCR_GSW2TX                  = 1<<10,
	DSI_CMCR_GSR0TX                  = 1<<11,
	DSI_CMCR_GSR1TX                  = 1<<12,
	DSI_CMCR_GSR2TX                  = 1<<13,
	DSI_CMCR_GLWTX                   = 1<<14,
	DSI_CMCR_DSW0TX                  = 1<<16,
	DSI_CMCR_DSW1TX                  = 1<<17,
	DSI_CMCR_DSR0TX                  = 1<<18,
	DSI_CMCR_DLWTX                   = 1<<19,
	DSI_CMCR_MRDPS                   = 1<<24
} dsi_cmcr_flags_t;
#define DSI_CMCR_FLAGS_MASK            0x010F7F03
/**@}*/

/** @defgroup dsi_ghcr_values DSI_GHCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host generic header configuration
name     | bits  | description   | default
-------- | ----- | ------------- | -------
DT       | 0:5   | Type          | 0
VCID     | 6:7   | Channel       | 0
WCLSB    | 8:15  | WordCount LSB | 0
WCMSB    | 16:23 | WordCount MSB | 0
Reserved | 24:31 |               | 0
@{*/
#define DSI_GHCR_DT_MASK               0x0000003F
#define DSI_GHCR_DT_SHIFT              0
#define DSI_GHCR_VCID_MASK             0x00000003
#define DSI_GHCR_VCID_SHIFT            6
#define DSI_GHCR_WCLSB_MASK            0x000000FF
#define DSI_GHCR_WCLSB_SHIFT           8
#define DSI_GHCR_WCMSB_MASK            0x000000FF
#define DSI_GHCR_WCMSB_SHIFT           16
/**@}*/

/** @defgroup dsi_gpdr_values DSI_GPDR values
 * @ingroup dsi_host_registers
 * @brief DSI Host generic payload data
name  | bits  | description    | default
----- | ----- | -------------- | -------
DATA1 | 0:7   | Payload byte 1 | 0
DATA2 | 8:15  | Payload byte 2 | 0
DATA3 | 16:23 | Payload byte 3 | 0
DATA4 | 24:31 | Payload byte 4 | 0
@{*/
#define DSI_GPDR_DATA1_MASK            0x000000FF
#define DSI_GPDR_DATA1_SHIFT           0
#define DSI_GPDR_DATA2_MASK            0x000000FF
#define DSI_GPDR_DATA2_SHIFT           8
#define DSI_GPDR_DATA3_MASK            0x000000FF
#define DSI_GPDR_DATA3_SHIFT           16
#define DSI_GPDR_DATA4_MASK            0x000000FF
#define DSI_GPDR_DATA4_SHIFT           24
/**@}*/

/** @defgroup dsi_gpsr_values DSI_GPSR values
 * @ingroup dsi_host_registers
 * @brief DSI Host generic packet status
name     | bits | description              | default
-------- | ---- | ------------------------ | -------
CMDFE    | 0    | Command FIFO empty       | 1
CMDFF    | 1    | Command FIFO full        | 0
PWRFE    | 2    | Payload write FIFO empty | 1
PWRFF    | 3    | Payload write FIFO full  | 0
PRDFE    | 4    | Payload read FIFO empty  | 1
PRDFF    | 5    | Payload read FIFO full   | 0
RCB      | 6    | Read command busy        | 0
Reserved | 7:31 |                          | 0
@{*/
typedef enum {
	DSI_GPSR_CMDFE                   = 1<<0,
	DSI_GPSR_CMDFF                   = 1<<1,
	DSI_GPSR_PWRFE                   = 1<<2,
	DSI_GPSR_PWRFF                   = 1<<3,
	DSI_GPSR_PRDFE                   = 1<<4,
	DSI_GPSR_PRDFF                   = 1<<5,
	DSI_GPSR_RCB                     = 1<<6
} dsi_gpsr_flags_t;
#define DSI_GPSR_FLAGS_MASK            0x0000007F
/**@}*/

/** @defgroup dsi_tccr0_values DSI_TCCR0 values
 * @ingroup dsi_host_registers
 * @brief DSI Host timeout counter configuration 0
name       | bits  | description                             | default
---------- | ----- | --------------------------------------- | -------
LPRX_TOCNT | 0:15  | Low-power reception timeout counter     | 0
HSTX_TOCNT | 16:31 | High-speed transmission timeout counter | 0
@{*/
#define DSI_TCCR0_LPRX_TOCNT_MASK      0x0000FFFF
#define DSI_TCCR0_LPRX_TOCNT_SHIFT     0
#define DSI_TCCR0_HSTX_TOCNT_MASK      0x0000FFFF
#define DSI_TCCR0_HSTX_TOCNT_SHIFT     16
/**@}*/

/** @defgroup dsi_tccr1_values DSI_TCCR1 values
 * @ingroup dsi_host_registers
 * @brief DSI Host timeout counter configuration 1
name       | bits  | description                     | default
---------- | ----- | ------------------------------- | -------
HSRD_TOCNT | 0:15  | High-speed read timeout counter | 0
Reserved   | 16:31 |                                 | 0
@{*/
#define DSI_TCCR1_HSRD_TOCNT_MASK      0x0000FFFF
#define DSI_TCCR1_HSRD_TOCNT_SHIFT     0
/**@}*/

/** @defgroup dsi_tccr2_values DSI_TCCR2 values
 * @ingroup dsi_host_registers
 * @brief DSI Host timeout counter configuration 2
name       | bits  | description                    | default
---------- | ----- | ------------------------------ | -------
LPRD_TOCNT | 0:15  | Low-power read timeout counter | 0
Reserved   | 16:31 |                                | 0
@{*/
#define DSI_TCCR2_LPRD_TOCNT_MASK      0x0000FFFF
#define DSI_TCCR2_LPRD_TOCNT_SHIFT     0
/**@}*/

/** @defgroup dsi_tccr3_values DSI_TCCR3 values
 * @ingroup dsi_host_registers
 * @brief DSI Host timeout counter configuration 3
name       | bits  | description                      | default
---------- | ----- | -------------------------------- | -------
HSWR_TOCNT | 0:15  | High-speed write timeout counter | 0
Reserved   | 16:23 |                                  | 0
PM         | 24    | Presp mode                       | 0
Reserved   | 25:31 |                                  | 0
@{*/
typedef enum {
	DSI_TCCR3_PM                     = 1<<24
} dsi_tccr3_flags_t;
#define DSI_TCCR3_FLAGS_MASK           0x01000000
#define DSI_TCCR3_HSWR_TOCNT_MASK      0x0000FFFF
#define DSI_TCCR3_HSWR_TOCNT_SHIFT     0
/**@}*/

/** @defgroup dsi_tccr4_values DSI_TCCR4 values
 * @ingroup dsi_host_registers
 * @brief DSI Host timeout counter configuration 4
name       | bits  | description                     | default
---------- | ----- | ------------------------------- | -------
LPWR_TOCNT | 0:15  | Low-power write timeout counter | 0
Reserved   | 16:31 |                                 | 0
@{*/
#define DSI_TCCR4_LPWR_TOCNT_MASK      0x0000FFFF
#define DSI_TCCR4_LPWR_TOCNT_SHIFT     0
/**@}*/

/** @defgroup dsi_tccr5_values DSI_TCCR5 values
 * @ingroup dsi_host_registers
 * @brief DSI Host timeout counter configuration 5
name      | bits  | description                     | default
--------- | ----- | ------------------------------- | -------
BTA_TOCNT | 0:15  | Bus-turn-around timeout counter | 0
Reserved  | 16:31 |                                 | 0
@{*/
#define DSI_TCCR5_BTA_TOCNT_MASK       0x0000FFFF
#define DSI_TCCR5_BTA_TOCNT_SHIFT      0
/**@}*/

/** @defgroup dsi_clcr_values DSI_CLCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host clock lane configuration
name     | bits | description                  | default
-------- | ---- | ---------------------------- | -------
DPCC     | 0    | D-PHY clock control          | 0
ACR      | 1    | Automatic clock lane control | 0
Reserved | 2:31 |                              | 0
@{*/
typedef enum {
	DSI_CLCR_DPCC                    = 1<<0,
	DSI_CLCR_ACR                     = 1<<1
} dsi_clcr_flags_t;
#define DSI_CLCR_FLAGS_MASK            0x00000003
/**@}*/

/** @defgroup dsi_cltcr_values DSI_CLTCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host clock lane timer configuration
name       | bits  | description                  | default
---------- | ----- | ---------------------------- | -------
LP2HS_TIME | 0:9   | Low-power to high-speed time | 0
Reserved   | 10:15 |                              | 0
HS2LP_TIME | 16:25 | High-speed to low-power time | 0
Reserved   | 26:31 |                              | 0
@{*/
#define DSI_CLTCR_LP2HS_TIME_MASK      0x000003FF
#define DSI_CLTCR_LP2HS_TIME_SHIFT     0
#define DSI_CLTCR_HS2LP_TIME_MASK      0x000003FF
#define DSI_CLTCR_HS2LP_TIME_SHIFT     16
/**@}*/

/** @defgroup dsi_dltcr_values DSI_DLTCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host data lane timer configuration
name       | bits  | description                  | default
---------- | ----- | ---------------------------- | -------
MRD_TIME   | 0:14  | Maximum read time            | 0
Reserved   | 15    |                              | 0
LP2HS_TIME | 16:23 | Low-power to high-speed time | 0
HS2LP_TIME | 24:31 | High-speed To low-power time | 0
@{*/
#define DSI_DLTCR_MRD_TIME_MASK        0x00007FFF
#define DSI_DLTCR_MRD_TIME_SHIFT       0
#define DSI_DLTCR_LP2HS_TIME_MASK      0x000000FF
#define DSI_DLTCR_LP2HS_TIME_SHIFT     16
#define DSI_DLTCR_HS2LP_TIME_MASK      0x000000FF
#define DSI_DLTCR_HS2LP_TIME_SHIFT     24
/**@}*/

/** @defgroup dsi_pctlr_values DSI_PCTLR values
 * @ingroup dsi_host_registers
 * @brief DSI Host PHY control
name     | bits | description    | default
-------- | ---- | -------------- | -------
Reserved | 0    |                | 0
DEN      | 1    | Digital enable | 0
CKE      | 2    | Clock enable   | 0
Reserved | 3:31 |                | 0
@{*/
typedef enum {
	DSI_PCTLR_DEN                    = 1<<1,
	DSI_PCTLR_CKE                    = 1<<2
} dsi_pctlr_flags_t;
#define DSI_PCTLR_FLAGS_MASK           0x00000006
/**@}*/

/** @defgroup dsi_pconfr_values DSI_PCONFR values
 * @ingroup dsi_host_registers
 * @brief DSI Host PHY configuration
name     | bits  | description     | default
-------- | ----- | --------------- | -------
NL       | 0:1   | Number of lanes | 1
Reserved | 2:7   |                 | 0
SW_TIME  | 8:15  | Stop wait time  | 0
Reserved | 16:31 |                 | 0
@{*/
#define DSI_PCONFR_NL_MASK             0x00000003
#define DSI_PCONFR_NL_SHIFT            0
#define DSI_PCONFR_SW_TIME_MASK        0x000000FF
#define DSI_PCONFR_SW_TIME_SHIFT       8
typedef enum {
	DSI_PCONFR_NL_1LANE = 0,
	DSI_PCONFR_NL_2LANE = 1
} dsi_pconfr_nl_t;
/**@}*/

/** @defgroup dsi_pucr_values DSI_PUCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host PHY ULPS control
name     | bits | description                | default
-------- | ---- | -------------------------- | -------
URCL     | 0    | ULPS request on clock lane | 0
UECL     | 1    | ULPS exit on clock lane    | 0
URDL     | 2    | ULPS request on data lane  | 0
UEDL     | 3    | ULPS exit on data lane     | 0
Reserved | 4:31 |                            | 0
@{*/
typedef enum {
	DSI_PUCR_URCL                    = 1<<0,
	DSI_PUCR_UECL                    = 1<<1,
	DSI_PUCR_URDL                    = 1<<2,
	DSI_PUCR_UEDL                    = 1<<3
} dsi_pucr_flags_t;
#define DSI_PUCR_FLAGS_MASK            0x0000000F
/**@}*/

/** @defgroup dsi_pttcr_values DSI_PTTCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host PHY TX triggers configuration
name     | bits | description          | default
-------- | ---- | -------------------- | -------
TX_TRIG  | 0:3  | Transmission trigger | 0
Reserved | 4:31 |                      | 0
@{*/
#define DSI_PTTCR_TX_TRIG_MASK         0x0000000F
#define DSI_PTTCR_TX_TRIG_SHIFT        0
typedef enum {
	DSI_PTTCR_TX_TRIG_1 = 0x1,
	DSI_PTTCR_TX_TRIG_2 = 0x2,
	DSI_PTTCR_TX_TRIG_3 = 0x4,
	DSI_PTTCR_TX_TRIG_4 = 0x8
} dsi_pttcr_tx_t;
/**@}*/

/** @defgroup dsi_psr_values DSI_PSR values
 * @ingroup dsi_host_registers
 * @brief DSI Host PHY status
name     | bits | description                | default
-------- | ---- | -------------------------- | -------
Reserved | 0    |                            | 0
PD       | 1    | PHY direction              | 0
PSSC     | 2    | PHY stop state clock lane  | 0
UANC     | 3    | ULPS active not clock lane | 1
PSS0     | 4    | PHY stop state lane 0      | 0
UAN0     | 5    | ULPS active not lane 1     | 1
RUE0     | 6    | RX ULPS escape lane 0      | 0
PSS1     | 7    | PHY stop state lane 1      | 0
UAN1     | 8    | ULPS active not lane 1     | 1
Reserved | 9:31 |                            | 10
@{*/
typedef enum {
	DSI_PSR_PD                       = 1<<1,
	DSI_PSR_PSSC                     = 1<<2,
	DSI_PSR_UANC                     = 1<<3,
	DSI_PSR_PSS0                     = 1<<4,
	DSI_PSR_UAN0                     = 1<<5,
	DSI_PSR_RUE0                     = 1<<6,
	DSI_PSR_PSS1                     = 1<<7,
	DSI_PSR_UAN1                     = 1<<8
} dsi_psr_flags_t;
#define DSI_PSR_FLAGS_MASK             0x000001FE
/**@}*/

/** @defgroup dsi_isr0_values DSI_ISR0 values
 * @ingroup dsi_host_registers
 * @brief DSI Host interrupt and status 0
name     | bits  | description          | default
-------- | ----- | -------------------- | -------
AE0      | 0     | Acknowledge error 0  | 0
AE1      | 1     | Acknowledge error 1  | 0
AE2      | 2     | Acknowledge error 2  | 0
AE3      | 3     | Acknowledge error 3  | 0
AE4      | 4     | Acknowledge error 4  | 0
AE5      | 5     | Acknowledge error 5  | 0
AE6      | 6     | Acknowledge error 6  | 0
AE7      | 7     | Acknowledge error 7  | 0
AE8      | 8     | Acknowledge error 8  | 0
AE9      | 9     | Acknowledge error 9  | 0
AE10     | 10    | Acknowledge error 10 | 0
AE11     | 11    | Acknowledge error 11 | 0
AE12     | 12    | Acknowledge error 12 | 0
AE13     | 13    | Acknowledge error 13 | 0
AE14     | 14    | Acknowledge error 14 | 0
AE15     | 15    | Acknowledge error 15 | 0
PE0      | 16    | PHY error 0          | 0
PE1      | 17    | PHY error 1          | 0
PE2      | 18    | PHY error 2          | 0
PE3      | 19    | PHY error 3          | 0
PE4      | 20    | PHY error 4          | 0
Reserved | 21:31 |                      | 0
@{*/
typedef enum {
	DSI_ISR0_AE0                     = 1<<0,
	DSI_ISR0_AE1                     = 1<<1,
	DSI_ISR0_AE2                     = 1<<2,
	DSI_ISR0_AE3                     = 1<<3,
	DSI_ISR0_AE4                     = 1<<4,
	DSI_ISR0_AE5                     = 1<<5,
	DSI_ISR0_AE6                     = 1<<6,
	DSI_ISR0_AE7                     = 1<<7,
	DSI_ISR0_AE8                     = 1<<8,
	DSI_ISR0_AE9                     = 1<<9,
	DSI_ISR0_AE10                    = 1<<10,
	DSI_ISR0_AE11                    = 1<<11,
	DSI_ISR0_AE12                    = 1<<12,
	DSI_ISR0_AE13                    = 1<<13,
	DSI_ISR0_AE14                    = 1<<14,
	DSI_ISR0_AE15                    = 1<<15,
	DSI_ISR0_PE0                     = 1<<16,
	DSI_ISR0_PE1                     = 1<<17,
	DSI_ISR0_PE2                     = 1<<18,
	DSI_ISR0_PE3                     = 1<<19,
	DSI_ISR0_PE4                     = 1<<20
} dsi_isr0_flags_t;
#define DSI_ISR0_FLAGS_MASK            0x001FFFFF
/**@}*/

/** @defgroup dsi_isr1_values DSI_ISR1 values
 * @ingroup dsi_host_registers
 * @brief DSI Host interrupt and status 1
name     | bits  | description                     | default
-------- | ----- | ------------------------------- | -------
TOHSTX   | 0     | Timeout high-speed transmission | 0
TOLPRX   | 1     | Timeout low-power reception     | 0
ECCSE    | 2     | ECC single-bit error            | 0
ECCME    | 3     | ECC multi-bit error             | 0
CRCE     | 4     | CRC error                       | 0
PSE      | 5     | Packet size error               | 0
EOTPE    | 6     | EoTp error                      | 0
LPWRE    | 7     | LTDC payload write error        | 0
GCWRE    | 8     | Generic command write error     | 0
GPWRE    | 9     | Generic payload write error     | 0
GPTXE    | 10    | Generic payload transmit error  | 0
GPRDE    | 11    | Generic payload read error      | 0
GPRXE    | 12    | Generic payload receive error   | 0
Reserved | 13:31 |                                 | 0
@{*/
typedef enum {
	DSI_ISR1_TOHSTX                  = 1<<0,
	DSI_ISR1_TOLPRX                  = 1<<1,
	DSI_ISR1_ECCSE                   = 1<<2,
	DSI_ISR1_ECCME                   = 1<<3,
	DSI_ISR1_CRCE                    = 1<<4,
	DSI_ISR1_PSE                     = 1<<5,
	DSI_ISR1_EOTPE                   = 1<<6,
	DSI_ISR1_LPWRE                   = 1<<7,
	DSI_ISR1_GCWRE                   = 1<<8,
	DSI_ISR1_GPWRE                   = 1<<9,
	DSI_ISR1_GPTXE                   = 1<<10,
	DSI_ISR1_GPRDE                   = 1<<11,
	DSI_ISR1_GPRXE                   = 1<<12
} dsi_isr1_flags_t;
#define DSI_ISR1_FLAGS_MASK            0x00001FFF
/**@}*/

/** @defgroup dsi_ier0_values DSI_IER0 values
 * @ingroup dsi_host_registers
 * @brief DSI Host interrupt enable 0
name     | bits  | description                           | default
-------- | ----- | ------------------------------------- | -------
AE0IE    | 0     | Acknowledge error 0 interrupt enable  | 0
AE1IE    | 1     | Acknowledge error 1 interrupt enable  | 0
AE2IE    | 2     | Acknowledge error 2 interrupt enable  | 0
AE3IE    | 3     | Acknowledge error 3 interrupt enable  | 0
AE4IE    | 4     | Acknowledge error 4 interrupt enable  | 0
AE5IE    | 5     | Acknowledge error 5 interrupt enable  | 0
AE6IE    | 6     | Acknowledge error 6 interrupt enable  | 0
AE7IE    | 7     | Acknowledge error 7 interrupt enable  | 0
AE8IE    | 8     | Acknowledge error 8 interrupt enable  | 0
AE9IE    | 9     | Acknowledge error 9 interrupt enable  | 0
AE10IE   | 10    | Acknowledge error 10 interrupt enable | 0
AE11IE   | 11    | Acknowledge error 11 interrupt enable | 0
AE12IE   | 12    | Acknowledge error 12 interrupt enable | 0
AE13IE   | 13    | Acknowledge error 13 interrupt enable | 0
AE14IE   | 14    | Acknowledge error 14 interrupt enable | 0
AE15IE   | 15    | Acknowledge error 15 interrupt enable | 0
PE0IE    | 16    | PHY error 0 interrupt enable          | 0
PE1IE    | 17    | PHY error 1 interrupt enable          | 0
PE2IE    | 18    | PHY error 2 interrupt enable          | 0
PE3IE    | 19    | PHY error 3 interrupt enable          | 0
PE4IE    | 20    | PHY error 4 interrupt enable          | 0
Reserved | 21:31 |                                       | 0
@{*/
typedef enum {
	DSI_IER0_AE0IE                   = 1<<0,
	DSI_IER0_AE1IE                   = 1<<1,
	DSI_IER0_AE2IE                   = 1<<2,
	DSI_IER0_AE3IE                   = 1<<3,
	DSI_IER0_AE4IE                   = 1<<4,
	DSI_IER0_AE5IE                   = 1<<5,
	DSI_IER0_AE6IE                   = 1<<6,
	DSI_IER0_AE7IE                   = 1<<7,
	DSI_IER0_AE8IE                   = 1<<8,
	DSI_IER0_AE9IE                   = 1<<9,
	DSI_IER0_AE10IE                  = 1<<10,
	DSI_IER0_AE11IE                  = 1<<11,
	DSI_IER0_AE12IE                  = 1<<12,
	DSI_IER0_AE13IE                  = 1<<13,
	DSI_IER0_AE14IE                  = 1<<14,
	DSI_IER0_AE15IE                  = 1<<15,
	DSI_IER0_PE0IE                   = 1<<16,
	DSI_IER0_PE1IE                   = 1<<17,
	DSI_IER0_PE2IE                   = 1<<18,
	DSI_IER0_PE3IE                   = 1<<19,
	DSI_IER0_PE4IE                   = 1<<20
} dsi_ier0_flags_t;
#define DSI_IER0_FLAGS_MASK            0x001FFFFF
/**@}*/

/** @defgroup dsi_ier1_values DSI_IER1 values
 * @ingroup dsi_host_registers
 * @brief DSI Host interrupt enable 1
name     | bits  | description                                      | default
-------- | ----- | ------------------------------------------------ | -------
TOHSTXIE | 0     | Timeout high-speed transmission interrupt enable | 0
TOLPRXIE | 1     | Timeout low-power reception interrupt enable     | 0
ECCSEIE  | 2     | ECC single-bit error interrupt enable            | 0
ECCMEIE  | 3     | ECC multi-bit error interrupt enable             | 0
CRCEIE   | 4     | CRC error interrupt enable                       | 0
PSEIE    | 5     | Packet size error interrupt enable               | 0
EOTPEIE  | 6     | EoTp error interrupt enable                      | 0
LPWREIE  | 7     | LTDC payload write error interrupt enable        | 0
GCWREIE  | 8     | Generic command write error interrupt enable     | 0
GPWREIE  | 9     | Generic payload write error interrupt enable     | 0
GPTXEIE  | 10    | Generic payload transmit error interrupt enable  | 0
GPRDEIE  | 11    | Generic payload read error interrupt enable      | 0
GPRXEIE  | 12    | Generic payload receive error interrupt enable   | 0
Reserved | 13:31 |                                                  | 0
@{*/
typedef enum {
	DSI_IER1_TOHSTXIE                = 1<<0,
	DSI_IER1_TOLPRXIE                = 1<<1,
	DSI_IER1_ECCSEIE                 = 1<<2,
	DSI_IER1_ECCMEIE                 = 1<<3,
	DSI_IER1_CRCEIE                  = 1<<4,
	DSI_IER1_PSEIE                   = 1<<5,
	DSI_IER1_EOTPEIE                 = 1<<6,
	DSI_IER1_LPWREIE                 = 1<<7,
	DSI_IER1_GCWREIE                 = 1<<8,
	DSI_IER1_GPWREIE                 = 1<<9,
	DSI_IER1_GPTXEIE                 = 1<<10,
	DSI_IER1_GPRDEIE                 = 1<<11,
	DSI_IER1_GPRXEIE                 = 1<<12
} dsi_ier1_flags_t;
#define DSI_IER1_FLAGS_MASK            0x00001FFF
/**@}*/

/** @defgroup dsi_fir0_values DSI_FIR0 values
 * @ingroup dsi_host_registers
 * @brief DSI Host force interrupt 0
name     | bits  | description                | default
-------- | ----- | -------------------------- | -------
FAE0     | 0     | Force acknowledge error 0  | 0
FAE1     | 1     | Force acknowledge error 1  | 0
FAE2     | 2     | Force acknowledge error 2  | 0
FAE3     | 3     | Force acknowledge error 3  | 0
FAE4     | 4     | Force acknowledge error 4  | 0
FAE5     | 5     | Force acknowledge error 5  | 0
FAE6     | 6     | Force acknowledge error 6  | 0
FAE7     | 7     | Force acknowledge error 7  | 0
FAE8     | 8     | Force acknowledge error 8  | 0
FAE9     | 9     | Force acknowledge error 9  | 0
FAE10    | 10    | Force acknowledge error 10 | 0
FAE11    | 11    | Force acknowledge error 11 | 0
FAE12    | 12    | Force acknowledge error 12 | 0
FAE13    | 13    | Force acknowledge error 13 | 0
FAE14    | 14    | Force acknowledge error 14 | 0
FAE15    | 15    | Force acknowledge error 15 | 0
FPE0     | 16    | Force PHY error 0          | 0
FPE1     | 17    | Force PHY error 1          | 0
FPE2     | 18    | Force PHY error 2          | 0
FPE3     | 19    | Force PHY error 3          | 0
FPE4     | 20    | Force PHY error 4          | 0
Reserved | 21:31 |                            | 0
@{*/
typedef enum {
	DSI_FIR0_FAE0                    = 1<<0,
	DSI_FIR0_FAE1                    = 1<<1,
	DSI_FIR0_FAE2                    = 1<<2,
	DSI_FIR0_FAE3                    = 1<<3,
	DSI_FIR0_FAE4                    = 1<<4,
	DSI_FIR0_FAE5                    = 1<<5,
	DSI_FIR0_FAE6                    = 1<<6,
	DSI_FIR0_FAE7                    = 1<<7,
	DSI_FIR0_FAE8                    = 1<<8,
	DSI_FIR0_FAE9                    = 1<<9,
	DSI_FIR0_FAE10                   = 1<<10,
	DSI_FIR0_FAE11                   = 1<<11,
	DSI_FIR0_FAE12                   = 1<<12,
	DSI_FIR0_FAE13                   = 1<<13,
	DSI_FIR0_FAE14                   = 1<<14,
	DSI_FIR0_FAE15                   = 1<<15,
	DSI_FIR0_FPE0                    = 1<<16,
	DSI_FIR0_FPE1                    = 1<<17,
	DSI_FIR0_FPE2                    = 1<<18,
	DSI_FIR0_FPE3                    = 1<<19,
	DSI_FIR0_FPE4                    = 1<<20
} dsi_fir0_flags_t;
#define DSI_FIR0_FLAGS_MASK            0x001FFFFF
/**@}*/

/** @defgroup dsi_fir1_values DSI_FIR1 values
 * @ingroup dsi_host_registers
 * @brief DSI Host force interrupt 1
name     | bits  | description                           | default
-------- | ----- | ------------------------------------- | -------
FTOHSTX  | 0     | Force timeout high-speed transmission | 0
FTOLPRX  | 1     | Force timeout low-power reception     | 0
FECCSE   | 2     | Force ECC single-bit error            | 0
FECCME   | 3     | Force ECC multi-bit error             | 0
FCRCE    | 4     | Force CRC error                       | 0
FPSE     | 5     | Force packet size error               | 0
FEOTPE   | 6     | Force EoTp error                      | 0
FLPWRE   | 7     | Force LTDC payload write error        | 0
FGCWRE   | 8     | Force generic command write error     | 0
FGPWRE   | 9     | Force generic payload write error     | 0
FGPTXE   | 10    | Force generic payload transmit error  | 0
FGPRDE   | 11    | Force generic payload read error      | 0
FGPRXE   | 12    | Force generic payload receive error   | 0
Reserved | 13:31 |                                       | 0
@{*/
typedef enum {
	DSI_FIR1_FTOHSTX                 = 1<<0,
	DSI_FIR1_FTOLPRX                 = 1<<1,
	DSI_FIR1_FECCSE                  = 1<<2,
	DSI_FIR1_FECCME                  = 1<<3,
	DSI_FIR1_FCRCE                   = 1<<4,
	DSI_FIR1_FPSE                    = 1<<5,
	DSI_FIR1_FEOTPE                  = 1<<6,
	DSI_FIR1_FLPWRE                  = 1<<7,
	DSI_FIR1_FGCWRE                  = 1<<8,
	DSI_FIR1_FGPWRE                  = 1<<9,
	DSI_FIR1_FGPTXE                  = 1<<10,
	DSI_FIR1_FGPRDE                  = 1<<11,
	DSI_FIR1_FGPRXE                  = 1<<12
} dsi_fir1_flags_t;
#define DSI_FIR1_FLAGS_MASK            0x00001FFF
/**@}*/

/** @defgroup dsi_vscr_values DSI_VSCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video shadow control
name     | bits | description     | default
-------- | ---- | --------------- | -------
EN       | 0    | Enable          | 0
Reserved | 1:7  |                 | 0
UR       | 8    | Update register | 0
Reserved | 9:31 |                 | 0
@{*/
typedef enum {
	DSI_VSCR_EN                      = 1<<0,
	DSI_VSCR_UR                      = 1<<8
} dsi_vscr_flags_t;
#define DSI_VSCR_FLAGS_MASK            0x00000101
/**@}*/

/** @defgroup dsi_lcvcidr_values DSI_LCVCIDR values
 * @ingroup dsi_host_registers
 * @brief DSI Host LTDC current VCID
name     | bits | description        | default
-------- | ---- | ------------------ | -------
VCID     | 0:1  | Virtual channel ID | 0
Reserved | 2:31 |                    | 0
@{*/
#define DSI_LCVCIDR_VCID_MASK          0x00000003
#define DSI_LCVCIDR_VCID_SHIFT         0
/**@}*/

/** @defgroup dsi_lcccr_values DSI_LCCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host LTDC current color coding
name     | bits | description           | default
-------- | ---- | --------------------- | -------
COLC     | 0:3  | Color coding          | 0
Reserved | 4:7  |                       | 0
LPE      | 8    | Loosely packed enable | 0
Reserved | 9:31 |                       | 0
@{*/
typedef enum {
	DSI_LCCCR_LPE                    = 1<<8
} dsi_lcccr_flags_t;
#define DSI_LCCCR_FLAGS_MASK           0x00000100
#define DSI_LCCCR_COLC_MASK            0x0000000F
#define DSI_LCCCR_COLC_SHIFT           0
/**@}*/

/** @defgroup dsi_lpmccr_values DSI_LPMCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host low-power mode current configuration
name     | bits  | description              | default
-------- | ----- | ------------------------ | -------
VLPSIZE  | 0:7   | VACT largest packet size | 0
Reserved | 8:15  |                          | 0
LPSIZE   | 16:23 | Largest packet size      | 0
Reserved | 24:31 |                          | 0
@{*/
#define DSI_LPMCCR_VLPSIZE_MASK        0x000000FF
#define DSI_LPMCCR_VLPSIZE_SHIFT       0
#define DSI_LPMCCR_LPSIZE_MASK         0x000000FF
#define DSI_LPMCCR_LPSIZE_SHIFT        16
/**@}*/

/** @defgroup dsi_vmccr_values DSI_VMCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video mode current configuration
name     | bits  | description                             | default
-------- | ----- | --------------------------------------- | -------
VMT      | 0:1   | Video mode type                         | 0
LPVSAE   | 2     | Low-power vertical sync time enable     | 0
LPVBPE   | 3     | Low-power vertical back-porch enable    | 0
LPVFPE   | 4     | Low-power vertical front-porch enable   | 0
LPVAE    | 5     | Low-power vertical active enable        | 0
LPHBPE   | 6     | Low-power horizontal back-porch enable  | 0
LPHFE    | 7     | Low-power horizontal front-porch enable | 0
FBTAAE   | 8     | Frame BTA acknowledge enable            | 0
LPCE     | 9     | Low-power command enable                | 0
Reserved | 10:31 |                                         | 0
@{*/
typedef enum {
	DSI_VMCCR_LPVSAE                 = 1<<2,
	DSI_VMCCR_LPVBPE                 = 1<<3,
	DSI_VMCCR_LPVFPE                 = 1<<4,
	DSI_VMCCR_LPVAE                  = 1<<5,
	DSI_VMCCR_LPHBPE                 = 1<<6,
	DSI_VMCCR_LPHFE                  = 1<<7,
	DSI_VMCCR_FBTAAE                 = 1<<8,
	DSI_VMCCR_LPCE                   = 1<<9
} dsi_vmccr_flags_t;
#define DSI_VMCCR_FLAGS_MASK           0x000003FC
#define DSI_VMCCR_VMT_MASK             0x00000003
#define DSI_VMCCR_VMT_SHIFT            0
/**@}*/

/** @defgroup dsi_vpccr_values DSI_VPCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video packet current configuration
name     | bits  | description       | default
-------- | ----- | ----------------- | -------
VPSIZE   | 0:13  | Video packet size | 0
Reserved | 14:31 |                   | 0
@{*/
#define DSI_VPCCR_VPSIZE_MASK          0x00003FFF
#define DSI_VPCCR_VPSIZE_SHIFT         0
/**@}*/

/** @defgroup dsi_vcccr_values DSI_VCCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video chunks current configuration
name     | bits  | description      | default
-------- | ----- | ---------------- | -------
NUMC     | 0:12  | Number of chunks | 0
Reserved | 13:31 |                  | 0
@{*/
#define DSI_VCCCR_NUMC_MASK            0x00001FFF
#define DSI_VCCCR_NUMC_SHIFT           0
/**@}*/

/** @defgroup dsi_vnpccr_values DSI_VNPCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video null packet current configuration
name     | bits  | description      | default
-------- | ----- | ---------------- | -------
NPSIZE   | 0:12  | Null packet size | 0
Reserved | 13:31 |                  | 0
@{*/
#define DSI_VNPCCR_NPSIZE_MASK         0x00001FFF
#define DSI_VNPCCR_NPSIZE_SHIFT        0
/**@}*/

/** @defgroup dsi_vhsaccr_values DSI_VHSACCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video HSA current configuration
name     | bits  | description                            | default
-------- | ----- | -------------------------------------- | -------
HSA      | 0:11  | Horizontal synchronism active duration | 0
Reserved | 12:31 |                                        | 0
@{*/
#define DSI_VHSACCR_HSA_MASK           0x00000FFF
#define DSI_VHSACCR_HSA_SHIFT          0
/**@}*/

/** @defgroup dsi_vhbpccr_values DSI_VHBPCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video HBP current configuration
name     | bits  | description                    | default
-------- | ----- | ------------------------------ | -------
HBP      | 0:11  | Horizontal back-porch duration | 0
Reserved | 12:31 |                                | 0
@{*/
#define DSI_VHBPCCR_HBP_MASK           0x00000FFF
#define DSI_VHBPCCR_HBP_SHIFT          0
/**@}*/

/** @defgroup dsi_vlccr_values DSI_VLCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video line current configuration
name     | bits  | description              | default
-------- | ----- | ------------------------ | -------
HLINE    | 0:14  | Horizontal line duration | 0
Reserved | 15:31 |                          | 0
@{*/
#define DSI_VLCCR_HLINE_MASK           0x00007FFF
#define DSI_VLCCR_HLINE_SHIFT          0
/**@}*/

/** @defgroup dsi_vvsaccr_values DSI_VVSACCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video VSA current configuration
name     | bits  | description                          | default
-------- | ----- | ------------------------------------ | -------
VSA      | 0:9   | Vertical synchronism active duration | 0
Reserved | 10:31 |                                      | 0
@{*/
#define DSI_VVSACCR_VSA_MASK           0x000003FF
#define DSI_VVSACCR_VSA_SHIFT          0
/**@}*/

/** @defgroup dsi_vvbpccr_values DSI_VVBPCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video VBP current configuration
name     | bits  | description                  | default
-------- | ----- | ---------------------------- | -------
VBP      | 0:9   | Vertical back-porch duration | 0
Reserved | 10:31 |                              | 0
@{*/
#define DSI_VVBPCCR_VBP_MASK           0x000003FF
#define DSI_VVBPCCR_VBP_SHIFT          0
/**@}*/

/** @defgroup dsi_vvfpccr_values DSI_VVFPCCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video VFP current configuration
name     | bits  | description                   | default
-------- | ----- | ----------------------------- | -------
VFP      | 0:9   | Vertical front-porch duration | 0
Reserved | 10:31 |                               | 0
@{*/
#define DSI_VVFPCCR_VFP_MASK           0x000003FF
#define DSI_VVFPCCR_VFP_SHIFT          0
/**@}*/

/** @defgroup dsi_vvaccr_values DSI_VVACCR values
 * @ingroup dsi_host_registers
 * @brief DSI Host video VA current configuration
name     | bits  | description              | default
-------- | ----- | ------------------------ | -------
VA       | 0:13  | Vertical active duration | 0
Reserved | 14:31 |                          | 0
@{*/
#define DSI_VVACCR_VA_MASK             0x00003FFF
#define DSI_VVACCR_VA_SHIFT            0
/**@}*/


/* --- DSI_WRAPPER register bits --- */

/** @defgroup dsi_wcfgr_values DSI_WCFGR values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper configuration
name     | bits | description        | default
-------- | ---- | ------------------ | -------
DSIM     | 0    | DSI mode           | 0
COLMUX   | 1:3  | Color multiplexing | 0
TESRC    | 4    | TE source          | 0
TEPOL    | 5    | TE polarity        | 0
AR       | 6    | Automatic refresh  | 0
VSPOL    | 7    | VSync polarity     | 0
Reserved | 8:31 |                    | 0
@{*/
typedef enum {
	DSI_WCFGR_DSIM                   = 1<<0,
	DSI_WCFGR_TESRC                  = 1<<4,
	DSI_WCFGR_TEPOL                  = 1<<5,
	DSI_WCFGR_AR                     = 1<<6,
	DSI_WCFGR_VSPOL                  = 1<<7
} dsi_wcfgr_flags_t;
#define DSI_WCFGR_FLAGS_MASK           0x000000F1
#define DSI_WCFGR_COLMUX_MASK          0x00000007
#define DSI_WCFGR_COLMUX_SHIFT         1
/**@}*/

/** @defgroup dsi_wcr_values DSI_WCR values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper control
name     | bits | description | default
-------- | ---- | ----------- | -------
COLM     | 0    | Color mode  | 0
SHTDN    | 1    | Shutdown    | 0
LTDCEN   | 2    | LTDC enable | 0
DSIEN    | 3    | DSI enable  | 0
Reserved | 4:31 |             | 0
@{*/
typedef enum {
	DSI_WCR_COLM                     = 1<<0,
	DSI_WCR_SHTDN                    = 1<<1,
	DSI_WCR_LTDCEN                   = 1<<2,
	DSI_WCR_DSIEN                    = 1<<3
} dsi_wcr_flags_t;
#define DSI_WCR_FLAGS_MASK             0x0000000F
/**@}*/

/** @defgroup dsi_wier_values DSI_WIER values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper interrupt enable
name     | bits  | description                      | default
-------- | ----- | -------------------------------- | -------
TEIE     | 0     | Tearing effect interrupt enable  | 0
ERIE     | 1     | End of refresh interrupt enable  | 0
Reserved | 2:8   |                                  | 0
PLLLIE   | 9     | PLL lock interrupt enable        | 0
PLLUIE   | 10    | PLL unlock interrupt enable      | 0
Reserved | 11:12 |                                  | 0
RRIE     | 13    | Regulator ready interrupt enable | 0
Reserved | 14:31 |                                  | 0
@{*/
typedef enum {
	DSI_WIER_TEIE                    = 1<<0,
	DSI_WIER_ERIE                    = 1<<1,
	DSI_WIER_PLLLIE                  = 1<<9,
	DSI_WIER_PLLUIE                  = 1<<10,
	DSI_WIER_RRIE                    = 1<<13
} dsi_wier_flags_t;
#define DSI_WIER_FLAGS_MASK            0x00002603
/**@}*/

/** @defgroup dsi_wisr_values DSI_WISR values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper interrupt and status
name     | bits  | description                    | default
-------- | ----- | ------------------------------ | -------
TEIF     | 0     | Tearing effect interrupt flag  | 0
ERIF     | 1     | End of refresh interrupt flag  | 0
BUSY     | 2     | Busy flag                      | 0
Reserved | 3:7   |                                | 0
PLLLS    | 8     | PLL lock status                | 0
PLLLIF   | 9     | PLL lock interrupt flag        | 0
PLLUIF   | 10    | PLL unlock interrupt flag      | 0
Reserved | 11    |                                | 0
RRS      | 12    | Regulator ready status         | 0
RRIF     | 13    | Regulator ready interrupt flag | 0
Reserved | 14:31 |                                | 0
@{*/
typedef enum {
	DSI_WISR_TEIF                    = 1<<0,
	DSI_WISR_ERIF                    = 1<<1,
	DSI_WISR_BUSY                    = 1<<2,
	DSI_WISR_PLLLS                   = 1<<8,
	DSI_WISR_PLLLIF                  = 1<<9,
	DSI_WISR_PLLUIF                  = 1<<10,
	DSI_WISR_RRS                     = 1<<12,
	DSI_WISR_RRIF                    = 1<<13
} dsi_wisr_flags_t;
#define DSI_WISR_FLAGS_MASK            0x00003707
/**@}*/

/** @defgroup dsi_wifcr_values DSI_WIFCR values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper interrupt flag clear
name     | bits  | description                          | default
-------- | ----- | ------------------------------------ | -------
CTEIF    | 0     | Clear tearing effect interrupt flag  | 0
CERIF    | 1     | Clear end of refresh interrupt flag  | 0
Reserved | 2:8   |                                      | 0
CPLLLIF  | 9     | Clear PLL lock interrupt flag        | 0
CPLLUIF  | 10    | Clear PLL unlock interrupt flag      | 0
Reserved | 11:12 |                                      | 0
CRRIF    | 13    | Clear regulator ready interrupt flag | 0
Reserved | 14:31 |                                      | 0
@{*/
typedef enum {
	DSI_WIFCR_CTEIF                  = 1<<0,
	DSI_WIFCR_CERIF                  = 1<<1,
	DSI_WIFCR_CPLLLIF                = 1<<9,
	DSI_WIFCR_CPLLUIF                = 1<<10,
	DSI_WIFCR_CRRIF                  = 1<<13
} dsi_wifcr_flags_t;
#define DSI_WIFCR_FLAGS_MASK           0x00002603
/**@}*/

/** @defgroup dsi_wpcr0_values DSI_WPCR0 values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper PHY configuration 0
name       | bits  | description                                      | default
---------- | ----- | ------------------------------------------------ | -------
UIX4       | 0:5   | Unit interval multiplied by 4                    | 0
SWCL       | 6     | Swap clock lane pins                             | 0
SWDL0      | 7     | Swap data lane 0 pins                            | 0
SWDL1      | 8     | Swap data lane 1 pins                            | 0
HSICL      | 9     | Invert high-speed data signal on clock lane      | 0
HSIDL0     | 10    | Invert the high-speed data signal on data lane 0 | 0
HSIDL1     | 11    | Invert the high-speed data signal on data lane 1 | 0
FTXSMCL    | 12    | Force in TX Stop mode the clock lane             | 0
FTXSMDL    | 13    | Force in TX Stop mode the data lanes             | 0
CDOFFDL    | 14    | Contention detection OFF on data lanes           | 0
Reserved   | 15    |                                                  | 0
TDDL       | 16    | Turn disable data lanes                          | 0
Reserved   | 17    |                                                  | 0
PDEN       | 18    | Pull-down enable                                 | 0
TCLKPREPEN | 19    | Custom time for tCLK-PREPARE enable              | 0
TCLKZEROEN | 20    | Custom time for tCLK-ZERO enable                 | 0
THSPREPEN  | 21    | Custom time for tHS-PREPARE enable               | 0
THSTRAILEN | 22    | Custom time for tHS-TRAIL enable                 | 0
THSZEROEN  | 23    | Custom time for tHS-ZERO enable                  | 0
TLPXDEN    | 24    | Custom time for tLPX for data lanes enable       | 0
THSEXITEN  | 25    | Custom time for tHS-EXIT enable                  | 0
TLPXCEN    | 26    | Custom time for tLPX for clock lane enable       | 0
TCLKPOSTEN | 27    | Custom time for tCLK-POST enable                 | 0
Reserved   | 28:31 |                                                  | 0
@{*/
typedef enum {
	DSI_WPCR0_SWCL                   = 1<<6,
	DSI_WPCR0_SWDL0                  = 1<<7,
	DSI_WPCR0_SWDL1                  = 1<<8,
	DSI_WPCR0_HSICL                  = 1<<9,
	DSI_WPCR0_HSIDL0                 = 1<<10,
	DSI_WPCR0_HSIDL1                 = 1<<11,
	DSI_WPCR0_FTXSMCL                = 1<<12,
	DSI_WPCR0_FTXSMDL                = 1<<13,
	DSI_WPCR0_CDOFFDL                = 1<<14,
	DSI_WPCR0_TDDL                   = 1<<16,
	DSI_WPCR0_PDEN                   = 1<<18,
	DSI_WPCR0_TCLKPREPEN             = 1<<19,
	DSI_WPCR0_TCLKZEROEN             = 1<<20,
	DSI_WPCR0_THSPREPEN              = 1<<21,
	DSI_WPCR0_THSTRAILEN             = 1<<22,
	DSI_WPCR0_THSZEROEN              = 1<<23,
	DSI_WPCR0_TLPXDEN                = 1<<24,
	DSI_WPCR0_THSEXITEN              = 1<<25,
	DSI_WPCR0_TLPXCEN                = 1<<26,
	DSI_WPCR0_TCLKPOSTEN             = 1<<27
} dsi_wpcr0_flags_t;
#define DSI_WPCR0_FLAGS_MASK           0x0FFD7FC0
#define DSI_WPCR0_UIX4_MASK            0x0000003F
#define DSI_WPCR0_UIX4_SHIFT           0
/**@}*/

/** @defgroup dsi_wpcr1_values DSI_WPCR1 values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper PHY configuration 1
name      | bits  | description                                                 | default
--------- | ----- | ----------------------------------------------------------- | -------
HSTXDCL   | 0:1   | High-speed transmission delay on clock lane                 | 0
HSTXDDL   | 2:3   | High-speed transmission delay on data lanes                 | 0
Reserved  | 4:5   |                                                             | 0
LPSRCCL   | 6:7   | Low-power transmission slew-rate compensation on clock lane | 0
LPSRCDL   | 8:9   | Low-power transmission slew-rate compensation on data lanes | 0
Reserved  | 10:11 |                                                             | 0
SDDC      | 12    | SDD control                                                 | 0
Reserved  | 13:15 |                                                             | 0
HSTXSRCCL | 16:17 | High-speed transmission slew-rate control on clock lane     | 0
HSTXSRCDL | 18:19 | High-speed transmission slew-rate control on data lanes     | 0
Reserved  | 20:21 |                                                             | 0
FLPRXLPM  | 22    | Forces LP receiver in low-power mode                        | 0
Reserved  | 23:24 |                                                             | 0
LPRXFT    | 25:26 | Low-power RX low-pass filtering tuning                      | 0
Reserved  | 27:31 |                                                             | 0
@{*/
typedef enum {
	DSI_WPCR1_SDDC                   = 1<<12,
	DSI_WPCR1_FLPRXLPM               = 1<<22
} dsi_wpcr1_flags_t;
#define DSI_WPCR1_FLAGS_MASK           0x00401000
#define DSI_WPCR1_HSTXDCL_MASK         0x00000003
#define DSI_WPCR1_HSTXDCL_SHIFT        0
#define DSI_WPCR1_HSTXDDL_MASK         0x00000003
#define DSI_WPCR1_HSTXDDL_SHIFT        2
#define DSI_WPCR1_LPSRCCL_MASK         0x00000003
#define DSI_WPCR1_LPSRCCL_SHIFT        6
#define DSI_WPCR1_LPSRCDL_MASK         0x00000003
#define DSI_WPCR1_LPSRCDL_SHIFT        8
#define DSI_WPCR1_HSTXSRCCL_MASK       0x00000003
#define DSI_WPCR1_HSTXSRCCL_SHIFT      16
#define DSI_WPCR1_HSTXSRCDL_MASK       0x00000003
#define DSI_WPCR1_HSTXSRCDL_SHIFT      18
#define DSI_WPCR1_LPRXFT_MASK          0x00000003
#define DSI_WPCR1_LPRXFT_SHIFT         25
/**@}*/

/** @defgroup dsi_wpcr2_values DSI_WPCR2 values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper PHY configuration 2
name     | bits  | description  | default
-------- | ----- | ------------ | -------
TCLKPREP | 0:7   | tCLK-PREPARE | 0
TCLKZERO | 8:15  | tCLK-ZERO    | 0
THSPREP  | 16:23 | tHS-PREPARE  | 0
THSTRAIL | 24:31 | tHSTRAIL     | 0
@{*/
#define DSI_WPCR2_TCLKPREP_MASK        0x000000FF
#define DSI_WPCR2_TCLKPREP_SHIFT       0
#define DSI_WPCR2_TCLKZERO_MASK        0x000000FF
#define DSI_WPCR2_TCLKZERO_SHIFT       8
#define DSI_WPCR2_THSPREP_MASK         0x000000FF
#define DSI_WPCR2_THSPREP_SHIFT        16
#define DSI_WPCR2_THSTRAIL_MASK        0x000000FF
#define DSI_WPCR2_THSTRAIL_SHIFT       24
/**@}*/

/** @defgroup dsi_wpcr3_values DSI_WPCR3 values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper PHY configuration 3
name    | bits  | description          | default
------- | ----- | -------------------- | -------
THSZERO | 0:7   | tHS-ZERO             | 0
TLPXD   | 8:15  | tLPX for data lanes  | 0
THSEXIT | 16:23 | tHSEXIT              | 0
TLPXC   | 24:31 | tLPXC for clock lane | 0
@{*/
#define DSI_WPCR3_THSZERO_MASK         0x000000FF
#define DSI_WPCR3_THSZERO_SHIFT        0
#define DSI_WPCR3_TLPXD_MASK           0x000000FF
#define DSI_WPCR3_TLPXD_SHIFT          8
#define DSI_WPCR3_THSEXIT_MASK         0x000000FF
#define DSI_WPCR3_THSEXIT_SHIFT        16
#define DSI_WPCR3_TLPXC_MASK           0x000000FF
#define DSI_WPCR3_TLPXC_SHIFT          24
/**@}*/

/** @defgroup dsi_wpcr4_values DSI_WPCR4 values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper PHY configuration 4
name     | bits | description | default
-------- | ---- | ----------- | -------
TCLKPOST | 0:7  | tCLK-POST   | 0
Reserved | 8:31 |             | 0
@{*/
#define DSI_WPCR4_TCLKPOST_MASK        0x000000FF
#define DSI_WPCR4_TCLKPOST_SHIFT       0
/**@}*/

/** @defgroup dsi_wrpcr_values DSI_WRPCR values
 * @ingroup dsi_wrapper_registers
 * @brief DSI wrapper regulator and PLL control
name     | bits  | description                | default
-------- | ----- | -------------------------- | -------
PLLEN    | 0     | PLL enable                 | 0
Reserved | 1     |                            | 0
NDIV     | 2:8   | PLL loop division factor   | 0
Reserved | 9:10  |                            | 0
IDF      | 11:14 | PLL input division factor  | 0
Reserved | 15    |                            | 0
ODF      | 16:17 | PLL output division factor | 0
Reserved | 18:23 |                            | 0
REGEN    | 24    | Regulator enable           | 0
Reserved | 25:31 |                            | 0
@{*/
typedef enum {
	DSI_WRPCR_PLLEN                  = 1<<0,
	DSI_WRPCR_REGEN                  = 1<<24
} dsi_wrpcr_flags_t;
#define DSI_WRPCR_FLAGS_MASK           0x01000001
#define DSI_WRPCR_NDIV_MASK            0x0000007F
#define DSI_WRPCR_NDIV_SHIFT           2
#define DSI_WRPCR_IDF_MASK             0x0000000F
#define DSI_WRPCR_IDF_SHIFT            11
#define DSI_WRPCR_ODF_MASK             0x00000003
#define DSI_WRPCR_ODF_SHIFT            16
typedef enum {
	DSI_WRPCR_ODF_DIV_1 = 0,
	DSI_WRPCR_ODF_DIV_2 = 1,
	DSI_WRPCR_ODF_DIV_4 = 2,
	DSI_WRPCR_ODF_DIV_8 = 3
} dsi_wrpcr_odf_t;
typedef enum {
	DSI_WRPCR_IDF_DIV_1 = 1,
	DSI_WRPCR_IDF_DIV_2 = 2,
	DSI_WRPCR_IDF_DIV_3 = 3,
	DSI_WRPCR_IDF_DIV_4 = 4,
	DSI_WRPCR_IDF_DIV_5 = 5,
	DSI_WRPCR_IDF_DIV_6 = 6,
	DSI_WRPCR_IDF_DIV_7 = 7
} dsi_wrpcr_idf_t;
/**@}*/


/** @cond */
#endif
/** @endcond */
/**@}*/
