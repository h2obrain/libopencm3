/** @defgroup rcc_file RCC peripheral API
 *
 * @ingroup peripheral_apis
 *
 * @section rcc_f4_api_ex Reset and Clock Control API.
 *
 * @brief <b>libopencm3 STM32F4xx Reset and Clock Control</b>
 *
 * @author @htmlonly &copy; @endhtmlonly 2013 Frantisek Burian <BuFran at seznam.cz>
 *
 * @date 18 Jun 2013
 *
 * This library supports the Reset and Clock Control System in the STM32 series
 * of ARM Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms @ref lgpl_license
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

const struct rcc_clock_scale rcc_hsi_configs[RCC_CLOCK_3V3_END] = {
	{ /* 84MHz */
		.pllm = 16,
		.plln = 336,
		.pllp = 4,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSI_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 84000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 96MHz */
		.pllm = 8,
		.plln = 96,
		.pllp = 2,
		.pllq = 4,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSI_CLK,
		.hpre = RCC_CFGR_HPRE_DIV_NONE,
		.ppre1 = RCC_CFGR_PPRE_DIV_2,
		.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
		.ahb_frequency  = 96000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 96000000
	},	
	{ /* 168MHz */
		.pllm = 16,
		.plln = 336,
		.pllp = 2,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSI_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 168000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 180MHz */
		.pllm = 16,
		.plln = 360,
		.pllp = 2,
		.pllq = 8,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSI_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 180000000,
		.apb1_frequency = 45000000,
		.apb2_frequency = 90000000,
	},
};

const struct rcc_clock_scale rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_END] = {
	{ /* 84MHz */
		.pllm = 8,
		.plln = 336,
		.pllp = 4,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 84000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 96MHz */
		.pllm = 4,
		.plln = 96,
		.pllp = 2,
		.pllq = 4,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_DIV_NONE,
		.ppre1 = RCC_CFGR_PPRE_DIV_2,
		.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
		.ahb_frequency  = 96000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 96000000
	},	
	{ /* 168MHz */
		.pllm = 8,
		.plln = 336,
		.pllp = 2,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 168000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 180MHz */
		.pllm = 8,
		.plln = 360,
		.pllp = 2,
		.pllq = 8,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 180000000,
		.apb1_frequency = 45000000,
		.apb2_frequency = 90000000,
	},
};

const struct rcc_clock_scale rcc_hse_12mhz_3v3[RCC_CLOCK_3V3_END] = {
	{ /* 84MHz */
		.pllm = 12,
		.plln = 336,
		.pllp = 4,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 84000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 96MHz */
		.pllm = 6,
		.plln = 96,
		.pllp = 2,
		.pllq = 4,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_DIV_NONE,
		.ppre1 = RCC_CFGR_PPRE_DIV_2,
		.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
		.ahb_frequency  = 96000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 96000000
	},	
	{ /* 168MHz */
		.pllm = 12,
		.plln = 336,
		.pllp = 2,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 168000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 180MHz */
		.pllm = 12,
		.plln = 360,
		.pllp = 2,
		.pllq = 8,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 180000000,
		.apb1_frequency = 45000000,
		.apb2_frequency = 90000000,
	},
};

const struct rcc_clock_scale rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_END] = {
	{ /* 84MHz */
		.pllm = 16,
		.plln = 336,
		.pllp = 4,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 84000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 96MHz */
		.pllm = 8,
		.plln = 96,
		.pllp = 2,
		.pllq = 4,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_DIV_NONE,
		.ppre1 = RCC_CFGR_PPRE_DIV_2,
		.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
		.ahb_frequency  = 96000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 96000000
	},	
	{ /* 168MHz */
		.pllm = 16,
		.plln = 336,
		.pllp = 2,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 168000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 180MHz */
		.pllm = 16,
		.plln = 360,
		.pllp = 2,
		.pllq = 8,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 180000000,
		.apb1_frequency = 45000000,
		.apb2_frequency = 90000000,
	},
};

const struct rcc_clock_scale rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_END] = {
	{ /* 84MHz */
		.pllm = 25,
		.plln = 336,
		.pllp = 4,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 84000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 96MHz */
		.pllm = 25,
		.plln = 192,
		.pllp = 2,
		.pllq = 4,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_DIV_NONE,
		.ppre1 = RCC_CFGR_PPRE_DIV_2,
		.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
		.ahb_frequency  = 96000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 96000000
	},	
	{ /* 168MHz */
		.pllm = 25,
		.plln = 336,
		.pllp = 2,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 168000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 180MHz */
		.pllm = 25,
		.plln = 360,
		.pllp = 2,
		.pllq = 8,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_5WS,
		.ahb_frequency  = 180000000,
		.apb1_frequency = 45000000,
		.apb2_frequency = 90000000,
	},
};


/**
 * Setup clocks to run from PLL.
 *
 * The arguments provide the pll source, multipliers, dividers, all that's
 * needed to establish a system clock.
 *
 * @param clock clock information structure.
 */
void rcc_clock_setup_pll(const struct rcc_clock_scale *clock)
{
	/* Enable internal high-speed oscillator (HSI). */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_HSI);

	/* Enable external high-speed oscillator (HSE). */
	if (clock->pll_source == RCC_CFGR_PLLSRC_HSE_CLK) {
		rcc_osc_on(RCC_HSE);
		rcc_wait_for_osc_ready(RCC_HSE);
	}

	/* Set the VOS scale mode */
	rcc_periph_clock_enable(RCC_PWR);
	pwr_set_vos_scale(clock->voltage_scale);

	/*
	 * Set prescalers for AHB, ADC, APB1, APB2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(clock->hpre);
	rcc_set_ppre1(clock->ppre1);
	rcc_set_ppre2(clock->ppre2);

	/* Disable PLL oscillator before changing its configuration. */
	rcc_osc_off(RCC_PLL);

	/* Configure the PLL oscillator. */
	if (clock->pll_source == RCC_CFGR_PLLSRC_HSE_CLK) {
		rcc_set_main_pll_hse(clock->pllm, clock->plln,
				clock->pllp, clock->pllq, clock->pllr);
	} else {
		rcc_set_main_pll_hsi(clock->pllm, clock->plln,
				clock->pllp, clock->pllq, clock->pllr);
	}

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Configure flash settings. */
	if (clock->flash_config & FLASH_ACR_DCEN) {
		flash_dcache_enable();
	} else {
		flash_dcache_disable();
	}
	if (clock->flash_config & FLASH_ACR_ICEN) {
		flash_icache_enable();
	} else {
		flash_icache_disable();
	}
	flash_set_ws(clock->flash_config);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL);

	/* Wait for PLL clock to be selected. */
	rcc_wait_for_sysclk_status(RCC_PLL);

	/* Set the peripheral clock frequencies used. */
	rcc_ahb_frequency  = clock->ahb_frequency;
	rcc_apb1_frequency = clock->apb1_frequency;
	rcc_apb2_frequency = clock->apb2_frequency;

	/* Disable internal high-speed oscillator. */
	if (clock->pll_source == RCC_CFGR_PLLSRC_HSE_CLK) {
		rcc_osc_off(RCC_HSI);
	}
}

/**
 * Setup clocks with the HSE.
 *
 * @deprecated replaced by rcc_clock_setup_pll as a drop in replacement.
 * @see rcc_clock_setup_pll which supports HSI as well as HSE, using the same
 * clock structures.
 */
void rcc_clock_setup_hse_3v3(const struct rcc_clock_scale *clock)
{
	rcc_clock_setup_pll(clock);
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the USART at base specified.
 * @param usart  Base address of USART to get clock frequency for.
 */
uint32_t rcc_get_usart_clk_freq(uint32_t usart)
{
	/* Handle values with selectable clocks. */
	if (usart == USART1_BASE || usart == USART6_BASE) {
		return rcc_apb2_frequency;
	} else {
		return rcc_apb1_frequency;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the Timer at base specified.
 * @param timer  Base address of TIM to get clock frequency for.
 */
uint32_t rcc_get_timer_clk_freq(uint32_t timer)
{
	/* Handle APB1 timer clocks. */
	if (timer >= TIM2_BASE && timer <= TIM14_BASE) {
		uint8_t ppre1 = (RCC_CFGR >> RCC_CFGR_PPRE1_SHIFT) & RCC_CFGR_PPRE1_MASK;
		return (ppre1 == RCC_CFGR_PPRE_DIV_NONE) ? rcc_apb1_frequency
			: 2 * rcc_apb1_frequency;
	} else {
		uint8_t ppre2 = (RCC_CFGR >> RCC_CFGR_PPRE2_SHIFT) & RCC_CFGR_PPRE2_MASK;
		return (ppre2 == RCC_CFGR_PPRE_DIV_NONE) ? rcc_apb2_frequency
			: 2 * rcc_apb2_frequency;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the I2C device at base specified.
 * @param i2c  Base address of I2C to get clock frequency for.
 */
uint32_t rcc_get_i2c_clk_freq(uint32_t i2c __attribute__((unused)))
{
	return rcc_apb1_frequency;
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the SPI device at base specified.
 * @param spi  Base address of SPI device to get clock frequency for (e.g. SPI1_BASE).
 */
uint32_t rcc_get_spi_clk_freq(uint32_t spi) {
	if (spi == SPI2_BASE || spi == SPI3_BASE) {
		return rcc_apb1_frequency;
	} else {
		return rcc_apb2_frequency;
	}
}
/**@}*/
