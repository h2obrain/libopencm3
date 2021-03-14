/** @defgroup rcc_file RCC peripheral API
 *
 * @ingroup peripheral_apis
 * This library supports the Reset and Clock Control System in the STM32 series
 * of ARM Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms @ref lgpl_license
 */

#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>

/**@{*/

// All PLL configurations without PLLM. PLLM should be set to the input clock
// frequency in MHz.
const struct rcc_clock_scale rcc_3v3[RCC_CLOCK_3V3_END] = {
	{ /* 216MHz */
		.plln = 432,
		.pllp = 2,
		.pllq = 9,
		.pllr = 0,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.vos_scale = PWR_SCALE1,
		.overdrive = 1,
		.flash_waitstates = 7,
		.ahb_frequency = 216000000,
		.apb1_frequency = 54000000,
		.apb2_frequency = 108000000,
	},
	{ /* 168MHz */
		.plln = 336,
		.pllp = 2,
		.pllq = 7,
		.pllr = 0,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.vos_scale = PWR_SCALE2,
		.overdrive = 1,
		.flash_waitstates = 5,
		.ahb_frequency = 168000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
	},
	{ /* 120MHz */
		.plln = 240,
		.pllp = 2,
		.pllq = 5,
		.pllr = 0,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.vos_scale = PWR_SCALE3,
		.overdrive = 0,
		.flash_waitstates = 3,
		.ahb_frequency = 120000000,
		.apb1_frequency = 30000000,
		.apb2_frequency = 60000000,
	},
	{ /* 72MHz */
		.plln = 144,
		.pllp = 2,
		.pllq = 3,
		.pllr = 0,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV4,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.vos_scale = PWR_SCALE3,
		.overdrive = 0,
		.flash_waitstates = 2,
		.ahb_frequency = 72000000,
		.apb1_frequency = 18000000,
		.apb2_frequency = 36000000,
	},
	{ /* 48MHz */
		.plln = 192,
		.pllp = 4,
		.pllq = 4,
		.pllr = 0,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_DIV2,
		.vos_scale = PWR_SCALE3,
		.overdrive = 0,
		.flash_waitstates = 1,
		.ahb_frequency = 48000000,
		.apb1_frequency = 24000000,
		.apb2_frequency = 24000000,
	},
	{ /* 24MHz */
		.plln = 192,
		.pllp = 8,
		.pllq = 4,
		.pllr = 0,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_NODIV,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.vos_scale = PWR_SCALE3,
		.overdrive = 0,
		.flash_waitstates = 0,
		.ahb_frequency = 24000000,
		.apb1_frequency = 24000000,
		.apb2_frequency = 24000000,
	}
};


void rcc_clock_setup_hse(const struct rcc_clock_scale *clock, uint32_t hse_mhz)
{
	uint8_t pllm = hse_mhz;

	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_HSI);

	/* Enable external high-speed oscillator. */
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);

	rcc_periph_clock_enable(RCC_PWR);
	pwr_set_vos_scale(clock->vos_scale);

	if (clock->overdrive) {
		pwr_enable_overdrive();
	}

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
	rcc_set_main_pll_hse(pllm, clock->plln,
			clock->pllp, clock->pllq, clock->pllr);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Configure flash settings. */
	flash_set_ws(clock->flash_waitstates);
	flash_art_enable();
	flash_prefetch_enable();

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL);

	/* Wait for PLL clock to be selected. */
	rcc_wait_for_sysclk_status(RCC_PLL);

	/* Set the clock frequencies used. */
	rcc_ahb_frequency = clock->ahb_frequency;
	rcc_apb1_frequency = clock->apb1_frequency;
	rcc_apb2_frequency = clock->apb2_frequency;

	/* Disable internal high-speed oscillator. */
	rcc_osc_off(RCC_HSI);
}

void rcc_clock_setup_hsi(const struct rcc_clock_scale *clock)
{
	uint8_t pllm = 16;

	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_HSI);

	rcc_periph_clock_enable(RCC_PWR);
	pwr_set_vos_scale(clock->vos_scale);

	if (clock->overdrive) {
		pwr_enable_overdrive();
	}

	/*
	 * Set prescalers for AHB, ADC, APB1, APB2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(clock->hpre);
	rcc_set_ppre1(clock->ppre1);
	rcc_set_ppre2(clock->ppre2);

	rcc_set_main_pll_hsi(pllm, clock->plln,
			     clock->pllp, clock->pllq, clock->pllr);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Configure flash settings. */
	flash_set_ws(clock->flash_waitstates);
	flash_art_enable();
	flash_prefetch_enable();

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL);

	/* Wait for PLL clock to be selected. */
	rcc_wait_for_sysclk_status(RCC_PLL);

	/* Set the clock frequencies used. */
	rcc_ahb_frequency = clock->ahb_frequency;
	rcc_apb1_frequency = clock->apb1_frequency;
	rcc_apb2_frequency = clock->apb2_frequency;
}

static uint32_t rcc_usart_i2c_clksel_freq(uint32_t apb_clk, uint8_t shift) {
	uint8_t clksel = (RCC_DCKCFGR2 >> shift) & RCC_DCKCFGR2_UARTxSEL_MASK;
	uint8_t hpre = (RCC_CFGR >> RCC_CFGR_HPRE_SHIFT) & RCC_CFGR_HPRE_MASK;
	switch (clksel) {
	case RCC_DCKCFGR2_UARTxSEL_PCLK:
		return apb_clk;
	case RCC_DCKCFGR2_UARTxSEL_SYSCLK:
		return rcc_ahb_frequency * rcc_get_div_from_hpre(hpre);
	/* This case is only valid for uarts, not for i2c! */
	case RCC_DCKCFGR2_UARTxSEL_LSE:
		return 32768;
	case RCC_DCKCFGR2_UARTxSEL_HSI:
		return 16000000U;
	}
	cm3_assert_not_reached();
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the USART at base specified.
 * @param usart  Base address of USART to get clock frequency for.
 */
uint32_t rcc_get_usart_clk_freq(uint32_t usart)
{
	/* F7 is highly configurable, every USART can be configured in DCKCFGR2. */
	if (usart == USART1_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb2_frequency, RCC_DCKCFGR2_UART1SEL_SHIFT);
	} else if (usart == USART2_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_USART2SEL_SHIFT);
	} else if (usart == USART3_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_USART3SEL_SHIFT);
	} else if (usart == UART4_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_UART4SEL_SHIFT);
	} else if (usart == UART5_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_UART5SEL_SHIFT);
	} else if (usart == USART6_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb2_frequency, RCC_DCKCFGR2_USART6SEL_SHIFT);
	} else if (usart == UART7_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_UART7SEL_SHIFT);
	} else {  /* UART8 */
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_UART8SEL_SHIFT);
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
		return (ppre1 == RCC_CFGR_PPRE_NODIV) ? rcc_apb1_frequency
			: 2 * rcc_apb1_frequency;
	} else {
		uint8_t ppre2 = (RCC_CFGR >> RCC_CFGR_PPRE2_SHIFT) & RCC_CFGR_PPRE2_MASK;
		return (ppre2 == RCC_CFGR_PPRE_NODIV) ? rcc_apb2_frequency
			: 2 * rcc_apb2_frequency;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the I2C device at base specified.
 * @param i2c  Base address of I2C to get clock frequency for.
 */
uint32_t rcc_get_i2c_clk_freq(uint32_t i2c __attribute__((unused)))
{
	if (i2c == I2C1_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_I2C1SEL_SHIFT);
	} else if (i2c == I2C2_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_I2C2SEL_SHIFT);
	} else if (i2c == I2C3_BASE) {
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_I2C3SEL_SHIFT);
	} else {  /* I2C4 */
		return rcc_usart_i2c_clksel_freq(rcc_apb1_frequency, RCC_DCKCFGR2_I2C4SEL_SHIFT);
	}
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
