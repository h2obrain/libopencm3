/** @addtogroup rcc_file
 * @ingroup peripheral_apis
 */

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Federico Ruiz-Ugalde <memeruiz at gmail dot com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
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

#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>

/**@{*/

/* Set the default clock frequencies after reset. */
uint32_t rcc_ahb_frequency = 16000000;
uint32_t rcc_apb1_frequency = 16000000;
uint32_t rcc_apb2_frequency = 16000000;

/* see f4/rcc.c and f7/rcc.c */

void rcc_osc_ready_int_clear(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CIR |= RCC_CIR_PLLRDYC;
		break;
	case RCC_HSE:
		RCC_CIR |= RCC_CIR_HSERDYC;
		break;
	case RCC_HSI:
		RCC_CIR |= RCC_CIR_HSIRDYC;
		break;
	case RCC_LSE:
		RCC_CIR |= RCC_CIR_LSERDYC;
		break;
	case RCC_LSI:
		RCC_CIR |= RCC_CIR_LSIRDYC;
		break;
	case RCC_PLLSAI:
		RCC_CIR |= RCC_CIR_PLLSAIRDYC;
		break;
	case RCC_PLLI2S:
		RCC_CIR |= RCC_CIR_PLLI2SRDYC;
		break;
	}
}

void rcc_osc_ready_int_enable(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CIR |= RCC_CIR_PLLRDYIE;
		break;
	case RCC_HSE:
		RCC_CIR |= RCC_CIR_HSERDYIE;
		break;
	case RCC_HSI:
		RCC_CIR |= RCC_CIR_HSIRDYIE;
		break;
	case RCC_LSE:
		RCC_CIR |= RCC_CIR_LSERDYIE;
		break;
	case RCC_LSI:
		RCC_CIR |= RCC_CIR_LSIRDYIE;
		break;
	case RCC_PLLSAI:
		RCC_CIR |= RCC_CIR_PLLSAIRDYIE;
		break;
	case RCC_PLLI2S:
		RCC_CIR |= RCC_CIR_PLLI2SRDYIE;
		break;
	}
}

void rcc_osc_ready_int_disable(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CIR &= ~RCC_CIR_PLLRDYIE;
		break;
	case RCC_HSE:
		RCC_CIR &= ~RCC_CIR_HSERDYIE;
		break;
	case RCC_HSI:
		RCC_CIR &= ~RCC_CIR_HSIRDYIE;
		break;
	case RCC_LSE:
		RCC_CIR &= ~RCC_CIR_LSERDYIE;
		break;
	case RCC_LSI:
		RCC_CIR &= ~RCC_CIR_LSIRDYIE;
		break;
	case RCC_PLLSAI:
		RCC_CIR &= ~RCC_CIR_PLLSAIRDYIE;
		break;
	case RCC_PLLI2S:
		RCC_CIR &= ~RCC_CIR_PLLI2SRDYIE;
		break;
	}
}

int rcc_osc_ready_int_flag(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		return ((RCC_CIR & RCC_CIR_PLLRDYF) != 0);
	case RCC_HSE:
		return ((RCC_CIR & RCC_CIR_HSERDYF) != 0);
	case RCC_HSI:
		return ((RCC_CIR & RCC_CIR_HSIRDYF) != 0);
	case RCC_LSE:
		return ((RCC_CIR & RCC_CIR_LSERDYF) != 0);
	case RCC_LSI:
		return ((RCC_CIR & RCC_CIR_LSIRDYF) != 0);
	case RCC_PLLSAI:
		return ((RCC_CIR & RCC_CIR_PLLSAIRDYF) != 0);
	case RCC_PLLI2S:
		return ((RCC_CIR & RCC_CIR_PLLI2SRDYF) != 0);
	}
	return 0;
}

void rcc_css_int_clear(void)
{
	RCC_CIR |= RCC_CIR_CSSC;
}

int rcc_css_int_flag(void)
{
	return ((RCC_CIR & RCC_CIR_CSSF) != 0);
}

bool rcc_is_osc_ready(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		return RCC_CR & RCC_CR_PLLRDY;
	case RCC_HSE:
		return RCC_CR & RCC_CR_HSERDY;
	case RCC_HSI:
		return RCC_CR & RCC_CR_HSIRDY;
	case RCC_LSE:
		return RCC_BDCR & RCC_BDCR_LSERDY;
	case RCC_LSI:
		return RCC_CSR & RCC_CSR_LSIRDY;
	case RCC_PLLSAI:
		return RCC_CR & RCC_CR_PLLSAIRDY;
	case RCC_PLLI2S:
		return RCC_CR & RCC_CR_PLLI2SRDY;
	}
	return false;
}

void rcc_wait_for_osc_ready(enum rcc_osc osc)
{
	while (!rcc_is_osc_ready(osc));
}

void rcc_wait_for_sysclk_status(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		while (((RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK) !=
			RCC_CFGR_SWS_PLL);
		break;
	case RCC_HSE:
		while (((RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK) !=
			RCC_CFGR_SWS_HSE);
		break;
	case RCC_HSI:
		while (((RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK) !=
			RCC_CFGR_SWS_HSI);
		break;
	default:
		/* Shouldn't be reached. */
		break;
	}
}

void rcc_osc_on(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CR |= RCC_CR_PLLON;
		break;
	case RCC_HSE:
		RCC_CR |= RCC_CR_HSEON;
		break;
	case RCC_HSI:
		RCC_CR |= RCC_CR_HSION;
		break;
	case RCC_LSE:
		RCC_BDCR |= RCC_BDCR_LSEON;
		break;
	case RCC_LSI:
		RCC_CSR |= RCC_CSR_LSION;
		break;
	case RCC_PLLSAI:
		RCC_CR |= RCC_CR_PLLSAION;
		break;
	case RCC_PLLI2S:
		RCC_CR |= RCC_CR_PLLI2SON;
		break;
	}
}

void rcc_osc_off(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CR &= ~RCC_CR_PLLON;
		break;
	case RCC_HSE:
		RCC_CR &= ~RCC_CR_HSEON;
		break;
	case RCC_HSI:
		RCC_CR &= ~RCC_CR_HSION;
		break;
	case RCC_LSE:
		RCC_BDCR &= ~RCC_BDCR_LSEON;
		break;
	case RCC_LSI:
		RCC_CSR &= ~RCC_CSR_LSION;
		break;
	case RCC_PLLSAI:
		RCC_CR &= ~RCC_CR_PLLSAION;
		break;
	case RCC_PLLI2S:
		RCC_CR &= ~RCC_CR_PLLI2SON;
		break;
	}
}

void rcc_css_enable(void)
{
	RCC_CR |= RCC_CR_CSSON;
}

void rcc_css_disable(void)
{
	RCC_CR &= ~RCC_CR_CSSON;
}

void rcc_pllsai_enable(void)
{
	RCC_CR |= RCC_CR_PLLSAION;
}

void rcc_pllsai_disable(void)
{
	RCC_CR &= ~RCC_CR_PLLSAION;
}

bool rcc_pllsai_ready(void)
{
	return (RCC_CR & RCC_CR_PLLSAIRDY) != 0;
}

/**
 * Set the dividers for the PLLI2S clock outputs
 * @param n valid range depends on target device, check your RefManual.
 * @param r valid range is 2..7
 */
void rcc_plli2s_config(uint16_t n, uint8_t r)
{
	RCC_PLLI2SCFGR = (
	  ((n & RCC_PLLI2SCFGR_PLLI2SN_MASK) << RCC_PLLI2SCFGR_PLLI2SN_SHIFT) |
	  ((r & RCC_PLLI2SCFGR_PLLI2SR_MASK) << RCC_PLLI2SCFGR_PLLI2SR_SHIFT));
}

/**
 * Set the dividers for the PLLSAI clock outputs
 * divider p is not available on all parts, pass 0 if not supported.
 * @param n valid range is 49..432
 * @param p 0 if unused, @ref rcc_pllsaicfgr_pllsaip
 * @param q valid range is 2..15
 * @param r valid range is 2..7
 * @sa rcc_pllsai_postscalers
 */
void rcc_pllsai_config(uint16_t n, uint16_t p, uint16_t q, uint16_t r)
{
	RCC_PLLSAICFGR = (
	  ((n & RCC_PLLSAICFGR_PLLSAIN_MASK) << RCC_PLLSAICFGR_PLLSAIN_SHIFT) |
	  ((p & RCC_PLLSAICFGR_PLLSAIP_MASK) << RCC_PLLSAICFGR_PLLSAIP_SHIFT) |
	  ((q & RCC_PLLSAICFGR_PLLSAIQ_MASK) << RCC_PLLSAICFGR_PLLSAIQ_SHIFT) |
	  ((r & RCC_PLLSAICFGR_PLLSAIR_MASK) << RCC_PLLSAICFGR_PLLSAIR_SHIFT));
}

#if defined(STM32F7)
/* Backwards compatibility */
#define RCC_DCKCFGR						RCC_DCKCFGR1
#define RCC_DCKCFGR_PLLSAIDIVR_MASK		RCC_DCKCFGR1_PLLSAIDIVR_MASK
#define RCC_DCKCFGR_PLLSAIDIVR_SHIFT	RCC_DCKCFGR1_PLLSAIDIVR_SHIFT
#define RCC_DCKCFGR_PLLSAIDIVQ_MASK	 	RCC_DCKCFGR1_PLLSAIDIVQ_MASK
#define RCC_DCKCFGR_PLLSAIDIVQ_SHIFT	RCC_DCKCFGR1_PLLSAIDIVQ_SHIFT
#endif

/**
 * Set the dedicated dividers after the PLLSAI configuration.
 *
 * @param q dedicated PLLSAI divider, for either A or B
 * @param r dedicated LCD-TFT divider, see LTDC
 * @sa rcc_pllsai_config
 */
void rcc_pllsai_postscalers(uint8_t q, uint8_t r)
{
	uint32_t reg32 = RCC_DCKCFGR;
	reg32 &= ((RCC_DCKCFGR_PLLSAIDIVR_MASK << RCC_DCKCFGR_PLLSAIDIVR_SHIFT)
		| (RCC_DCKCFGR_PLLSAIDIVQ_MASK << RCC_DCKCFGR_PLLSAIDIVQ_SHIFT));
	RCC_DCKCFGR = reg32 | ((q << RCC_DCKCFGR_PLLSAIDIVQ_SHIFT) |
		(r << RCC_DCKCFGR_PLLSAIDIVR_SHIFT));
}


void rcc_set_sysclk_source(uint32_t clk)
{
	uint32_t reg32;

	reg32 = RCC_CFGR;
	reg32 &= ~(RCC_CFGR_SW_MASK << RCC_CFGR_SW_SHIFT);
	RCC_CFGR = (reg32 | (clk << RCC_CFGR_SW_SHIFT));
}

void rcc_set_pll_source(uint32_t pllsrc)
{
	uint32_t reg32;

	reg32 = RCC_PLLCFGR;
	reg32 &= ~(1 << 22);
	RCC_PLLCFGR = (reg32 | (pllsrc << 22));
}

void rcc_set_ppre2(uint32_t ppre2)
{
	uint32_t reg32;

	reg32 = RCC_CFGR;
	reg32 &= ~(RCC_CFGR_PPRE2_MASK << RCC_CFGR_PPRE2_SHIFT);
	RCC_CFGR = (reg32 | (ppre2 << RCC_CFGR_PPRE2_SHIFT));
}

void rcc_set_ppre1(uint32_t ppre1)
{
	uint32_t reg32;

	reg32 = RCC_CFGR;
	reg32 &= ~(RCC_CFGR_PPRE1_MASK << RCC_CFGR_PPRE1_SHIFT);
	RCC_CFGR = (reg32 | (ppre1 << RCC_CFGR_PPRE1_SHIFT));
}

void rcc_set_hpre(uint32_t hpre)
{
	uint32_t reg32;

	reg32 = RCC_CFGR;
	reg32 &= ~(RCC_CFGR_HPRE_MASK << RCC_CFGR_HPRE_SHIFT);
	RCC_CFGR = (reg32 | (hpre << RCC_CFGR_HPRE_SHIFT));
}

void rcc_set_rtcpre(uint32_t rtcpre)
{
	uint32_t reg32;

	reg32 = RCC_CFGR;
	reg32 &= ~(RCC_CFGR_RTCPRE_MASK << RCC_CFGR_RTCPRE_SHIFT);
	RCC_CFGR = (reg32 | (rtcpre << RCC_CFGR_RTCPRE_SHIFT));
}

/**
 * Reconfigures the main PLL for a HSI source.
 * Any reserved bits are kept at their reset values.
 * @param pllm Divider for the main PLL input clock
 * @param plln Main PLL multiplication factor for VCO
 * @param pllp Main PLL divider for main system clock
 * @param pllq Main PLL divider for USB OTG FS, SDMMC & RNG
 * @param pllr Main PLL divider for DSI (for parts without DSI, provide 0 here)
 */
void rcc_set_main_pll_hsi(uint32_t pllm, uint32_t plln, uint32_t pllp,
			  uint32_t pllq, uint32_t pllr)
{
	/* Use reset value if not legal, for parts without pllr */
	if (pllr < 2) {
		pllr = 2;
	}
	RCC_PLLCFGR = 0 | /* HSI */
		((pllm & RCC_PLLCFGR_PLLM_MASK) << RCC_PLLCFGR_PLLM_SHIFT) |
		((plln & RCC_PLLCFGR_PLLN_MASK) << RCC_PLLCFGR_PLLN_SHIFT) |
		((((pllp >> 1) - 1) & RCC_PLLCFGR_PLLP_MASK) << RCC_PLLCFGR_PLLP_SHIFT) |
		((pllq & RCC_PLLCFGR_PLLQ_MASK) << RCC_PLLCFGR_PLLQ_SHIFT) |
		((pllr & RCC_PLLCFGR_PLLR_MASK) << RCC_PLLCFGR_PLLR_SHIFT);
}

/**
 * Reconfigures the main PLL for a HSE source.
 * Any reserved bits are kept at their reset values.
 * @param pllm Divider for the main PLL input clock
 * @param plln Main PLL multiplication factor for VCO
 * @param pllp Main PLL divider for main system clock
 * @param pllq Main PLL divider for USB OTG FS, SDMMC & RNG
 * @param pllr Main PLL divider for DSI (for parts without DSI, provide 0 here)
 */
void rcc_set_main_pll_hse(uint32_t pllm, uint32_t plln, uint32_t pllp,
			  uint32_t pllq, uint32_t pllr)
{
	/* Use reset value if not legal, for parts without pllr */
	if (pllr < 2) {
		pllr = 2;
	}
	RCC_PLLCFGR = RCC_PLLCFGR_PLLSRC | /* HSE */
		((pllm & RCC_PLLCFGR_PLLM_MASK) << RCC_PLLCFGR_PLLM_SHIFT) |
		((plln & RCC_PLLCFGR_PLLN_MASK) << RCC_PLLCFGR_PLLN_SHIFT) |
		((((pllp >> 1) - 1) & RCC_PLLCFGR_PLLP_MASK) << RCC_PLLCFGR_PLLP_SHIFT) |
		((pllq & RCC_PLLCFGR_PLLQ_MASK) << RCC_PLLCFGR_PLLQ_SHIFT) |
		((pllr & RCC_PLLCFGR_PLLR_MASK) << RCC_PLLCFGR_PLLR_SHIFT);
}

uint32_t rcc_system_clock_source(void)
{
	/* Return the clock source which is used as system clock. */
	return (RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK;
}

/* see f4/rcc.c and f7/rcc.c */

