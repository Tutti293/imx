#include <config.h>
#include <common.h>
#include <asm/io.h>
#include "include/mx6.h"
#include "include/mx6_pins.h"
#include "include/iomux-v3.h"
#include "include/mx6dl_pins.h"
//#include <asm/errno.h>
#include <miiphy.h>
//#include <asm/imx-common/boot_mode.h>
#include <asm/spl.h>
#if ENABLE_I2C_MXC
#include <i2c.h>
#endif

#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif

#ifdef CONFIG_ARCH_MMU
#include <asm/mmu.h>
#include <asm/arch/mmu.h>
#endif

#ifdef CONFIG_CMD_CLOCK
#include <asm/clock.h>
#endif

#ifdef CONFIG_IMX_ECSPI
#include <imx_spi.h>
#endif

#ifdef CONFIG_IMX_UDC
#include <usb/imx_udc.h>
#endif

#include "detect.h"
#undef  IO_ADDRESS
#define IO_ADDRESS(a)                         (a)
#define OALPAtoUA(a)                          (a)
//#define MX6_IO_ADDRESS(a)                   (a)
#define MX6SL_USB_ANALOG_DIGPROG            0x280
#define MX6_USB_ANALOG_DIGPROG              0x260
#define IMX_CHIP_REVISION_1_0                0x10
#define IMX_CHIP_REVISION_1_1                0x11
#define IMX_CHIP_REVISION_1_2                0x12
#define IMX_CHIP_REVISION_1_3                0x13
#define IMX_CHIP_REVISION_UNKNOWN            0xff
#define IMX_CHIP_REVISION_1_0_STRING         "1.0"
#define IMX_CHIP_REVISION_1_1_STRING         "1.1"
#define IMX_CHIP_REVISION_1_2_STRING         "1.2"
#define IMX_CHIP_REVISION_1_3_STRING         "1.3"

unsigned int  __mxc_cpu_type;
//char imx_silicon_rev[32];
static int cpu_silicon_rev = -1;

u32 spl_boot_device(void)
{
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned int gpr10_boot = readl(&psrc->gpr10) & (1 << 28);
	unsigned reg = gpr10_boot ? readl(&psrc->gpr9) : readl(&psrc->sbmr1);
	unsigned int bmode = readl(&psrc->sbmr2);

	/*
	 * Check for BMODE if serial downloader is enabled
	 * BOOT_MODE - see IMX6DQRM Table 8-1
	 */
	if ((((bmode >> 24) & 0x03)  == 0x01) || /* Serial Downloader */
		(gpr10_boot && (reg == 1)))
		return BOOT_DEVICE_UART;
	/* BOOT_CFG1[7:4] - see IMX6DQRM Table 8-8 */
	switch ((reg & 0x000000FF) >> 4) {
	 /* EIM: See 8.5.1, Table 8-9 */
	case 0x0:
		/* BOOT_CFG1[3]: NOR/OneNAND Selection */
		if ((reg & 0x00000008) >> 3)
			return BOOT_DEVICE_ONENAND;
		else
			return BOOT_DEVICE_NOR;
		break;
	/* SATA: See 8.5.4, Table 8-20 */
	case 0x2:
		return BOOT_DEVICE_SATA;
	/* Serial ROM: See 8.5.5.1, Table 8-22 */
	case 0x3:
		/* BOOT_CFG4[2:0] */
		switch ((reg & 0x07000000) >> 24) {
		case 0x0 ... 0x4:
			return BOOT_DEVICE_SPI;
		case 0x5 ... 0x7:
			return BOOT_DEVICE_I2C;
		}
		break;
	/* SD/eSD: 8.5.3, Table 8-15  */
	case 0x4:
	case 0x5:
		return BOOT_DEVICE_MMC1;
	/* MMC/eMMC: 8.5.3 */
	case 0x6:
	case 0x7:
		return BOOT_DEVICE_MMC1;
	/* NAND Flash: 8.5.2 */
	case 0x8 ... 0xf:
		return BOOT_DEVICE_NAND;
	}
	return BOOT_DEVICE_NONE;
}

void mxc_set_cpu_type(unsigned int type)
{
        __mxc_cpu_type = type;
}

static int mx6_get_srev(void)
{
  void *anatop = (void *) MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	u32 rev;
	if (cpu_is_mx6sl())
		rev = __raw_readl(anatop + MX6SL_USB_ANALOG_DIGPROG);
	else
		rev = __raw_readl(anatop + MX6_USB_ANALOG_DIGPROG);

	rev &= 0xff;

	if (rev == 0)
		return IMX_CHIP_REVISION_1_0;
	else if (rev == 1)
		return IMX_CHIP_REVISION_1_1;
	else if (rev == 2)
		return IMX_CHIP_REVISION_1_2;

	return IMX_CHIP_REVISION_UNKNOWN;
}

/*
 * Returns:
 *	the silicon revision of the cpu
 */
int mx6q_revision(void)
{
	if (!cpu_is_mx6q())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = mx6_get_srev();

	return cpu_silicon_rev;
}

/*
 * Returns:
 *	the silicon revision of the cpu
 */
int mx6dl_revision(void)
{
	if (!cpu_is_mx6dl())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = mx6_get_srev();

	return cpu_silicon_rev;
}

int mx6sl_revision(void)
{
        if (!cpu_is_mx6sl())
                return -EINVAL;

        if (cpu_silicon_rev == -1)
                cpu_silicon_rev = mx6_get_srev();

        return cpu_silicon_rev;
}


void mx6_set_cpu_type(void)
{
        u32 cpu_type = readl( IO_ADDRESS(ANATOP_BASE_ADDR + 0x280));
        cpu_type >>= 16;
        if (cpu_type == 0x60) {
                mxc_set_cpu_type(MXC_CPU_MX6SL);
		// sprintf(imx_silicon_rev, "i.MX6SoloLite-r%d", mx6sl_revision());
                return;
        }

        cpu_type = readl(IO_ADDRESS(ANATOP_BASE_ADDR + 0x260));
        cpu_type >>= 16;
        if (cpu_type == 0x63) {
                mxc_set_cpu_type(MXC_CPU_MX6Q);
		// sprintf(imx_silicon_rev,"i.MX6Q-r%d", mx6q_revision());
        } else if (cpu_type == 0x61) {
                mxc_set_cpu_type(MXC_CPU_MX6DL);
		// sprintf(imx_silicon_rev,"i.MX6DL/SOLO-r%d", mx6dl_revision());
        } else
                printf("Unknown CPU type: %x\n", cpu_type);
}
int verify_mx6q(void)
{
 return( ((readl(IO_ADDRESS(ANATOP_BASE_ADDR + 0x260))>>16)==0x63) ? 1: 0);
}

int verify_mx6dl(void)
{
  return( ((readl(IO_ADDRESS(ANATOP_BASE_ADDR + 0x260))>>16)==0x61) ? 1: 0);
}

#if 0
#undef cpu_is_mx6q()
#undef cpu_is_mx6q()   
#undef cpu_is_mx6dl()  
#undef cpu_is_mx6sl()  


int cpu_is_mx6q(void)
{
  return(mxc_cpu_type == MXC_CPU_MX6Q);
}

int cpu_is_mx6dl(void)
{
  return(mxc_cpu_type == MXC_CPU_MX6DL);
}

int cpu_is_mx6sl(void)
{  
  return(mxc_cpu_type == MXC_CPU_MX6SL);  
}
#endif

unsigned long GetRAMSize(void)
{
	// Bootloader will always use 15 rows.
	unsigned long mdctl;
	unsigned long mdmisc;
	unsigned long row, col, width, bank;
	unsigned long size;
	// Note: only calculates for 1ChipSelect:
	mdctl = *((unsigned long*)OALPAtoUA(0x21B0000));
	mdmisc= *((unsigned long*)OALPAtoUA(0x21B0018));
	row = 1<<(((mdctl >> 24)&0x7)+11);
	col = 1<<(((mdctl >> 20)&0x7)+9);
	switch((mdctl >> 16)&0x3)
	{
		case 0:
			width = 2;
			break;
		case 2:
			width = 8;
			break;
		case 1:
		default:
			width = 4;
			break;
	}
	if ( mdmisc & 0x20)
		bank = 4;
	else
		bank = 8;

	size = ( bank*row*col*width);
#if defined CONFIG_DDR_16BIT
	if( mdctl & 0x000F0000 == 0x00080000 )
	  size = size >> 1;
#endif

//	printf("GetRAMSize:\n - mdctl: %lu\n - mdmisc: %lu\n - row: %lu\n - col: %lu\n - width: %lu\n - bank: %lu\n - size: %lu\n", 
//							mdctl, mdmisc, row, col, width, bank, size);

	return size;
}


#if 0
// now in ./lowlevel_init.S
#define CSP_BASE_REG_PA_MMDC0                     0x021b0000
int adjust_memctlr(void)
{
  unsigned volatile long *lp1 = (unsigned long *) 0x10000000; /* CSP_BASE_MEM_PA_DRAM_LOW         */
  unsigned volatile long *lp3 = (unsigned long *) 0x30000000; /* CSP_BASE_MEM_PA_DRAM_LOW + 512MB */
  unsigned volatile long *lp2 = (unsigned long *) 0x50000000; /* CSP_BASE_MEM_PA_DRAM_LOW + 1GB   */
  unsigned volatile long *mmdc0=(unsigned long *) CSP_BASE_REG_PA_MMDC0;
  unsigned volatile long *lpt, a,b,c,i;


  a = *lp1; // save copy  
  i = ~a;   // inverse a;
  
  if(!verify_mx6dl())// dual banks
  {
    b    = *lp2;    // save                     high address value
    *lp2 = i;       // write inverse of base to high address 
    c    = *lp1;    // read base
    if( c == a )    // base still the same ?
    {
      *lp2 = b;    // recover high address value      
      return 0;    
    }else
    {
#if 1
      *mmdc0=0x831A0000;      
      return(0); 
#endif
    }
  }else
  {
    b    = *lp3;    // save                     high address value
    *lp3 = i;       // write inverse of base to high address 
    c    = *lp3;    // read base
    if( c == a )    // base still the same ?
    {
      *lp3 = b;    // recover high address value      
      return 0;    
    }else
    {
#if 1
#if CONFIG_DDR_16BIT
      *mmdc0=0x83180000;      
#else
      *mmdc0=0x83190000;      
#endif
#endif
    }
  }
  return(0);  
}
#endif
