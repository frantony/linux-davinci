/*
 * virt2real board support, based on TI DaVinci DM365 EVM board support
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/mux.h>
#include <mach/common.h>
#include <mach/serial.h>
#include <linux/platform_data/mmc-davinci.h>
#include <mach/gpio.h>

#include "davinci.h"
#include "dm365_spi.h"

static int card_always_present(int module)
{
	return 1;
}

static int card_always_rw(int module)
{
	return 0;
}

static struct davinci_mmc_config virt2real_mmc_config = {
	.get_cd		= card_always_present,
	.get_ro		= card_always_rw,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
};

static void virt2real_mmc_configure(void)
{
	/*
	 * MMC/SD pins are multiplexed with GPIO and EMIF
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_SD1_CLK);
	davinci_cfg_reg(DM365_SD1_CMD);
	davinci_cfg_reg(DM365_SD1_DATA3);
	davinci_cfg_reg(DM365_SD1_DATA2);
	davinci_cfg_reg(DM365_SD1_DATA1);
	davinci_cfg_reg(DM365_SD1_DATA0);
}

static void dm365_ks8851_init(void)
{
	gpio_request(0, "KSZ8851");
	gpio_direction_input(0);
	davinci_cfg_reg(DM365_EVT18_SPI3_TX);
	davinci_cfg_reg(DM365_EVT19_SPI3_RX);
}

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

static void __init virt2real_map_io(void)
{
	dm365_init();
}

static struct davinci_spi_config ksz8851_mcspi_config = {
	.io_type = SPI_IO_TYPE_DMA,
	.c2tdelay = 0,
	.t2cdelay = 0
};

static struct spi_board_info ksz8851_snl_info[] __initdata = {
	{
		.modalias	= "ks8851",
		.bus_num	= 3,
		.chip_select	= 0,
		.max_speed_hz	= 24000000,
		.controller_data = &ksz8851_mcspi_config,
		.irq		= IRQ_DM365_GPIO0
	}
};

static struct davinci_spi_unit_desc virt2real_spi_udesc_KSZ8851 = {
	.spi_hwunit = 3,
	.chipsel = BIT(0),
	.irq = IRQ_DM365_SPIINT3_0,
	.dma_tx_chan = 18,
	.dma_rx_chan = 19,
	.dma_evtq = EVENTQ_3,
	.pdata = {
		.version = SPI_VERSION_1,
		.num_chipselect = 2,
		.intr_line = 0,
		.chip_sel = 0,
		.cshold_bug = 0,
		.dma_event_q = EVENTQ_3,
	}
};

static __init void virt2real_init(void)
{
	struct clk *aemif;

	aemif = clk_get(NULL, "aemif");
	if (IS_ERR(aemif))
		WARN("%s: unable to get AEMIF clock\n", __func__);
	else
		clk_prepare_enable(aemif);

	davinci_serial_init(&uart_config);

	virt2real_mmc_configure();
	/*
	 * actually mmc1 is used, not 0!
	 * we have to use davinci_setup_mmc(1, &virt2real_mmc_config);
	 * see /proc/iomem output for working driver
	 *   01d11000-01d11fff : da830-mmc.0
	 *     01d11000-01d11fff : mmc0
	davinci_setup_mmc(1, &virt2real_mmc_config);
	*/

	dm365_ks8851_init();
	davinci_init_spi(&virt2real_spi_udesc_KSZ8851,
		ARRAY_SIZE(ksz8851_snl_info), ksz8851_snl_info);
}

MACHINE_START(VIRT2REAL, "virt2real")
	.atag_offset = 0x100,
	.map_io = virt2real_map_io,
	.init_irq = davinci_irq_init,
	.init_time = davinci_timer_init,
	.init_machine = virt2real_init,
	.init_late = davinci_init_late,
	.dma_zone_size = SZ_128M,
	.restart = davinci_restart,
MACHINE_END
