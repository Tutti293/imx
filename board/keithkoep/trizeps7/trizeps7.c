/* 
 * trizeps 7 board file
 */
#include <common.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <linux/delay.h> // udelay(us) function
#include <asm/arch/sys_proto.h> // is_<cputype>() & get_cpu_type() functions, imx_ddr_size(), imx_get_mac_from_fuse - (/arch/arm/include/asm/mach-imx/sys_proto.h)
#include <net.h> // is_valid_ethaddr()
#include <env.h>
#include <spl.h>
#include <mmc.h>
#include <asm/arch/clock.h> // for MXC_ESDHC4_CLK
#include <asm/gpio.h> // for gpio_direction_x && gpio_set_value
#include "detect.h" // for GetRAMSize()
#include <malloc.h> // for free()
#include <miiphy.h> // eth phy (?)
#include <netdev.h> // for board_eth_init
#if ENABLE_I2C_MXC
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#endif

#include "trizeps_version.h"

#ifdef CONFIG_FSL_USDHC
#include <fsl_esdhc_imx.h> // sd4 mmc init
#endif

int gpio_get_value(unsigned gpio);
static void board_setup_i2c(unsigned int module_base);

DECLARE_GLOBAL_DATA_PTR;
#if 1
#define IOMUX_PAD_CTRL(name, pad_ctrl)       \
    NEW_PAD_CTRL(MX6Q_PAD_##name, pad_ctrl), \
    NEW_PAD_CTRL(MX6DL_PAD_##name, pad_ctrl)
#else
#define IOMUX_PAD_CTRL(name, pad_ctrl)         \
    NEW_PAD_CTRL(MX6DL_PAD_##name, pad_ctrl)
#endif

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP | \
    PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |   \
    PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP | \
    PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |  \
    PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | \
    PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |   \
    PAD_CTL_HYS)

#define I2C_PAD_CTRL (PAD_CTL_PUS_100K_UP |               \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS | \
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)


#ifdef CONFIG_FSL_USDHC
static struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = {
	{USDHC4_BASE_ADDR},
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};
#endif

static iomux_v3_cfg_t const uart2_pads[] = {
    IOMUX_PAD_CTRL(SD3_DAT4__UART2_RX_DATA, UART_PAD_CTRL),
    IOMUX_PAD_CTRL(SD3_DAT5__UART2_TX_DATA, UART_PAD_CTRL),
};

#if 0
static iomux_v3_cfg_t const sd1_pads[] = {
	IOMUX_PAD_CTRL(SD1_CLK__SD1_CLK,      USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_CMD__SD1_CMD,      USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT0__SD1_DATA0,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT1__SD1_DATA1,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT2__SD1_DATA2,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT3__SD1_DATA3,   USDHC_PAD_CTRL),
};
#endif

static iomux_v3_cfg_t const sd2_pads[] = {
	IOMUX_PAD_CTRL(SD2_CLK__SD2_CLK,      USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__SD2_CMD,      USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT0__SD2_DATA0,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT1__SD2_DATA1,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT2__SD2_DATA2,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT3__SD2_DATA3,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(GPIO_4__GPIO1_IO04,    NO_PAD_CTRL),
};

static iomux_v3_cfg_t const sd3_pads[] = {
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK,      USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD,      USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3,   USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const sd4_pads[] = {
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK,      USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD,      USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT4__SD4_DATA4,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT5__SD4_DATA5,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT6__SD4_DATA6,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT7__SD4_DATA7,   USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(NANDF_ALE__GPIO6_IO08, NO_PAD_CTRL), // sd4_reset (emmc)
	IOMUX_PAD_CTRL(NANDF_D7__GPIO2_IO07,  NO_PAD_CTRL), // sd4_cd (card detect)
};

static iomux_v3_cfg_t const enet_pads[] = {
    IOMUX_PAD_CTRL(EIM_BCLK__GPIO6_IO31,     NO_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_MDC__ENET_MDC,       ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_MDIO__ENET_MDIO,     ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_RXD0__ENET_RX_DATA0, ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_RXD1__ENET_RX_DATA1, ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_RX_ER__ENET_RX_ER,   ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_TX_EN__ENET_TX_EN,   ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_TXD0__ENET_TX_DATA0, ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_TXD1__ENET_TX_DATA1, ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(ENET_CRS_DV__ENET_RX_EN,  ENET_PAD_CTRL),
    IOMUX_PAD_CTRL(GPIO_16__ENET_REF_CLK,    ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const i2c1_pads[] = {
	IOMUX_PAD_CTRL(CSI0_DAT8__I2C1_SDA, I2C_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT9__I2C1_SCL, I2C_PAD_CTRL),
};

static iomux_v3_cfg_t const i2c2_pads[] = {
	IOMUX_PAD_CTRL(KEY_ROW3__I2C2_SDA, I2C_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_COL3__I2C2_SCL, I2C_PAD_CTRL),
};

int dram_init(void) {
    gd->ram_size = GetRAMSize();   
    return 0;
}

static void setup_iomux_uart(void) {
    SETUP_IOMUX_PADS(uart2_pads);
}

#if 0
static void setup_iomux_sd1(void) {
	SETUP_IOMUX_PADS(sd1_pads);
}
#endif

static void setup_iomux_sd2(void) {
	SETUP_IOMUX_PADS(sd2_pads);
}

static void setup_iomux_sd3(void) {
	SETUP_IOMUX_PADS(sd3_pads);
}

static void setup_iomux_sd4(void) {
    SETUP_IOMUX_PADS(sd4_pads);
}

static void setup_iomux_i2c1(void) {
	SETUP_IOMUX_PADS(i2c1_pads);
}

static void setup_iomux_i2c2(void) {
	SETUP_IOMUX_PADS(i2c2_pads);
}

#define ETH_PHY_RESET	IMX_GPIO_NR(6, 31)
static void setup_iomux_enet(void) {
    SETUP_IOMUX_PADS(enet_pads);

    gpio_direction_output(ETH_PHY_RESET, 0);
	mdelay(10);
	gpio_set_value(ETH_PHY_RESET, 1);
	udelay(100);
}

void printSplBootDevice(void) {
    u32 booted_from = spl_boot_device(); // get_boot_device()
    printf("Boot device: ");
	switch (booted_from) {
	case BOOT_DEVICE_RAM:
		printf("RAM\n");
		break;
	case BOOT_DEVICE_MMC1:
		printf("MMC1\n");
		break;
	case BOOT_DEVICE_MMC2:
		printf("MMC2\n");
		break;
	case BOOT_DEVICE_MMC2_2:
		printf("MMC2_2\n");
		break;
	case BOOT_DEVICE_NAND:
		printf("NAND\n");
		break;
	case BOOT_DEVICE_ONENAND:
		printf("ONENAND\n");
		break;
	case BOOT_DEVICE_NOR:
		printf("NOR\n");
		break;
	case BOOT_DEVICE_UART:
		printf("UART\n");
		break;
	case BOOT_DEVICE_SATA:
		printf("SATA\n");
		break;
	case BOOT_DEVICE_I2C:
		printf("I2C\n");
		break;
	case BOOT_DEVICE_BOARD:
		printf("BOARD\n");
		break;
	case BOOT_DEVICE_NONE:
	default:
		printf("UNKNOWN\n");
		break;
	}
}

int board_init(void) {
    printSplBootDevice();
    get_trizeps_board_version();
    PrintTrizepsInfo();

#ifdef ENABLE_I2C_MXC
	board_setup_i2c(I2C1_BASE_ADDR);
	board_setup_i2c(I2C2_BASE_ADDR);
#endif
    return 0;
}

int board_late_init(void) {

//    printf("** board_late_init\n");
    unsigned char mac[6];
    unsigned char mac_wrong_order[6];
    int j=5;
    imx_get_mac_from_fuse(0, mac_wrong_order);
    for(int i=0; i<6; i++) {
        mac[i] = mac_wrong_order[j];
        j--;
    }

    if (is_valid_ethaddr(mac)) {
	    eth_env_set_enetaddr("ethaddr", mac);
	}

    printf("Use Mac: [");
    for (int i=0; i<6; i++) {
        printf("%02X", mac[i]);
        if (i<5) 
            printf(":"); 
        else 
            printf("]\n");
    }

    setup_iomux_enet();
    SetTrizepsEnvironment();
//    setup_iomux_backlight_standalone(1);
    return 0;
}

int board_early_init_f(void) {
    
    setup_iomux_uart();

    return 0;
}


//FIXME (these functions currently do not get called)
int board_mmc_get_env_dev(int devno) {
    printf("board_mmc_get_env_dev called: devno=%d\n", devno);
    return devno;
}

int mmc_map_to_kernel_blk(int devno) {
    printf("board_mmc_get_env_dev called: devno=%d\n", devno);
    return devno;
}

int mmc_get_env_devno(void) {
    printf("mmc_get_enc_devno called\n");
    return 1;
}
//FIXME

#define USDHC2_CD_GPIO  IMX_GPIO_NR(1, 4)
#define USDHC4_CD_GPIO  IMX_GPIO_NR(2, 7)
#define USDHC4_RST_GPIO IMX_GPIO_NR(6, 8)
int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret;

	switch (cfg->esdhc_base) 
	{
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = 1;
		break;
	case USDHC4_BASE_ADDR:
		ret = !gpio_get_value(USDHC4_CD_GPIO);
		break; 
	default:
		printf("board_mmc_getcd: default case -> error\n");
		ret=0;
	}
	printf("board_mmc_getcd 0x%x->%d\n\r", (unsigned int) cfg->esdhc_base, ret);	
	return ret;
}

int board_mmc_init(struct bd_info *bis) {
	// init MMC (SD4) w/o SPL support
	int ret=0;
	for(int i=0; i<CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch(i) {
			case 0:
				setup_iomux_sd4();
				usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
				usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
				usdhc_cfg[0].max_bus_width = 4;
				fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
				break;
			case 1:
				setup_iomux_sd2();
				usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
				usdhc_cfg[1].esdhc_base = USDHC2_BASE_ADDR;
				usdhc_cfg[1].max_bus_width = 4;
				fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
				break;
			case 2:
				setup_iomux_sd3();
				usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
				usdhc_cfg[2].esdhc_base = USDHC3_BASE_ADDR;
				usdhc_cfg[2].max_bus_width = 4;
				fsl_esdhc_initialize(bis, &usdhc_cfg[2]);
				break;
			default:
				printf("Warning: you configured more USDHC controllers"
				    "(%d) then supported by the board (%d)\n",
				    i + 1, CONFIG_SYS_FSL_USDHC_NUM);
				return -EINVAL;
		}
		if(ret) {
			printf("Error initilaizing USDHC#%d\n", i);
			return ret;
		}
	}

	gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
	return ret;
}

#ifdef ENABLE_I2C_MXC

#define CLKCTL_CCGR2 0x70 /* used to be in mx6.h */

static void board_setup_i2c(unsigned int module_base)
{
	unsigned int reg;
	switch (module_base) {
	case I2C1_BASE_ADDR:
		setup_iomux_i2c1();	
		/* Enable i2c clock */
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR2);
		reg |= 0xC0;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR2);
		break;
	case I2C2_BASE_ADDR:
		setup_iomux_i2c2();
		/* Enable i2c clock */
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR2);
		reg |= 0x300;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR2);

		break;
	default:
		break;
	}
}
#endif

#define ETH_PHY_MASK	((1 << 0x1))
int board_eth_init(struct bd_info *bis) {

    printf("\n*** Board Eth Init...\n");

	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct mii_dev *bus;
	struct phy_device *phydev;

    int ret = enable_fec_anatop_clock(0, ENET_50MHZ);
	if (ret)
		return ret;

	/* set gpr1[ENET_CLK_SEL] */
	setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_ENET_CLK_SEL_MASK);

	setup_iomux_enet();

	bus = fec_get_miibus(IMX_FEC_BASE, -1);
	if (!bus)
		return -EINVAL;

	phydev = phy_find_by_mask(bus, ETH_PHY_MASK, PHY_INTERFACE_MODE_RMII);
	if (!phydev) {
		ret = -EINVAL;
		goto free_bus;
	}
	printf("using phy at address %d\n", phydev->addr);
	debug("using phy at address %d\n", phydev->addr);
	ret = fec_probe(bis, -1, IMX_FEC_BASE, bus, phydev);

	if (ret)
		goto free_phydev;
	return 0;

free_phydev:
	free(phydev);
free_bus:
	free(bus);
	return ret;
}

#if defined(CONFIG_VIDEO_IPUV3)
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrbits_le32(&iomux->gpr[2],
		     IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		     IOMUXC_GPR2_LVDS_CH1_MODE_MASK);
}

iomux_v3_cfg_t mxq_hdmi_pads[] = 
{
  NEW_PAD_CTRL(MX6Q_PAD_EIM_DA15__GPIO_3_15|MUX_MODE_SION,PAD_CTL_TRIZEPS_GPIO_PU),
  NEW_PAD_CTRL(MX6Q_PAD_EIM_A16__GPIO_2_22 |MUX_MODE_SION,PAD_CTL_TRIZEPS_GPIO_PU),
  MX6Q_PAD_EIM_EB2__HDMI_TX_DDC_SCL,
  MX6Q_PAD_EIM_D16__HDMI_TX_DDC_SDA
};

iomux_v3_cfg_t mxdl_hdmi_pads[] = 
{
  NEW_PAD_CTRL(MX6DL_PAD_EIM_DA15__GPIO_3_15|MUX_MODE_SION,PAD_CTL_TRIZEPS_GPIO_PU),
  NEW_PAD_CTRL(MX6DL_PAD_EIM_A16__GPIO_2_22 |MUX_MODE_SION,PAD_CTL_TRIZEPS_GPIO_PU),
  MX6DL_PAD_EIM_EB2__HDMI_TX_DDC_SCL,
  MX6DL_PAD_EIM_D16__HDMI_TX_DDC_SDA
};

static void do_enable_hdmi(struct display_info_t const *dev)
{  
  if ( !verify_mx6q())
    mxc_iomux_v3_setup_multiple_pads(mxdl_hdmi_pads, ARRAY_SIZE( mxdl_hdmi_pads));
  else
    mxc_iomux_v3_setup_multiple_pads(mxq_hdmi_pads, ARRAY_SIZE( mxq_hdmi_pads));

  disable_lvds(dev);
  imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {
        {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= NULL,
	.mode	= {
		.name           = "IPAN7-EDT-18WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30000,
		.left_margin    = 89,
		.right_margin   = 39,
		.upper_margin   = 30,
		.lower_margin   = 12,
		.hsync_len      = 127,
		.vsync_len      = 3,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
                   } 
	},
	{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED,
	          }
	}
};
size_t display_count = ARRAY_SIZE(displays);

#define IPAN5_BL_EN_GPIO     	IMX_GPIO_NR(4, 20) //         MX6Q_PAD_DI0_PIN4__GPIO_4_20
#define IPAN_BL_EN_GPIO     	IMX_GPIO_NR(5, 20) // * 100:  MX6Q_PAD_CSI0_DATA_EN__GPIO_5_20

static void setup_iomux_backlight(int on)
{ 
  gpio_set_value(       IPAN5_BL_EN_GPIO, on);
  gpio_direction_output(IPAN5_BL_EN_GPIO, on); // BL-Enable

  gpio_set_value(        IPAN_BL_EN_GPIO, on);
  gpio_direction_output( IPAN_BL_EN_GPIO, on); // BL-Enable
}

static void set_reset_out( int level)
{
  if( TrizepsVersionFromTo(0,BOARD_TRIZEPS7_V1R2) )
  {  
    if ( verify_mx6q())
    {  
      printf("set_reset_out v1r2 mx6q %d\n", level);
      mxc_iomux_v3_setup_pad(NEW_PAD_CTRL(MX6Q_PAD_GPIO_6__GPIO_1_6|MUX_MODE_SION,PAD_CTL_TRIZEPS_GPIO_PU));    
    }else	
    {
      printf("set_reset_out v1r2 mx6dl %d\n", level);
      mxc_iomux_v3_setup_pad(NEW_PAD_CTRL(MX6DL_PAD_GPIO_6__GPIO_1_6|MUX_MODE_SION,PAD_CTL_TRIZEPS_GPIO_PU));    
    }    
    gpio_set_value(IMX_GPIO_NR(1, 6),       level);
    gpio_direction_output(IMX_GPIO_NR(1, 6),    1);
  }else
  { 
    if ( verify_mx6q())
    {	
      printf("set_reset_out v1r3 mx6q %d\n", level);
      mxc_iomux_v3_setup_pad(NEW_PAD_CTRL(MX6Q_PAD_GPIO_17__GPIO_7_12 |MUX_MODE_SION,PAD_CTL_TRIZEPS_GPIO_PU));    
    }else	
    {
      printf("set_reset_out v1r3 mx6dl %d\n", level);	
      mxc_iomux_v3_setup_pad(NEW_PAD_CTRL(MX6DL_PAD_GPIO_17__GPIO_7_12|MUX_MODE_SION,PAD_CTL_TRIZEPS_GPIO_PU));    
    }    
    gpio_set_value(IMX_GPIO_NR(7, 12),       level);
    gpio_direction_output(IMX_GPIO_NR(7, 12),    1);
  }
  // printf("set_reset_out v1r3 mx6q %d done\n", level);
}

int board_cfb_skip(void)
{
        char *s;
	ulong val=0;
	
	/* As default, don't skip cfb init */
        s=getenv("bmp_addr");	 

	if (s != NULL) {	
	  val = simple_strtoul(s, NULL, 16);
	}

	if( val )
	  printf("Skipping Logo");	
	else
	  setup_iomux_backlight(1);
	
	
	return val;
}


static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB_DI0 and LDB_DI1 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |= MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* Set LDB_DI0 and LDB_DI1 clk select to 3b'011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK |
		 MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET) |
	       (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0 <<
		MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW |
	      IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW |
	      IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG |
	      IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT |
	      IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG |
	      IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
	      IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0 |
	      IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg &= ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK |
		 IOMUXC_GPR3_HDMI_MUX_CTL_MASK);
	reg |= (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<
		IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET) |
	       (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<
		IOMUXC_GPR3_HDMI_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */





