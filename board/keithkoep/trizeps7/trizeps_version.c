#include <common.h>
#include <asm/io.h>
#include <asm/arch-imx/cpu.h>
//#include "include/mx6.h"
#include "include/mx6_pins.h"
#include "include/mx6dl_pins.h"
#include "include/iomux-v3.h"
//#include <asm/errno.h>
#include <miiphy.h>
#include <asm-generic/sections.h>

//#include <asm/imx-common/iomux-v3.h>
//#include <asm/imx-common/mxc_i2c.h>
//#include <asm/imx-common/boot_mode.h>
#include <asm/spl.h>
//#include <asm/imx-common/spi.h>
#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/sys_proto.h>
#if ENABLE_I2C_MXC
#include <i2c.h>
#endif
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/clock.h>
//#include <asm/imx-common/video.h>
#include <asm/arch/crm_regs.h>
#include <pca953x.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
//#include "../common/pfuze.h" //FIXME

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>

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

#ifdef CONFIG_IMX_THERMAL
#include <imx_thermal.h> // temp definitions
#include <thermal.h> // thermal_get_temp prototype
#endif

#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#include <part.h>
//#include <ext2fs.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <ubi_uboot.h>
#include <jffs2/load_kernel.h>
#include <asm/imx-common/boot_mode.h>
#endif

#include "detect.h"

#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
//#include <asm/imx-common/video.h>
#include <asm-generic/gpio.h>
#include "trizeps_version.h"

/* **************************************************************************************************** */
TRIZEPS_INFO TrizepsBoardVersion = 
{
  .trizeps_module = 7,
  .trizeps_sodimm = 200,
  .trizeps_extcon  = 1,
  .trizeps_resetout_gpio    = IMX_GPIO_NR(7,  12),
  .trizeps_hw_board_version = BOARD_TRIZEPS7_V1R2,
  .trizeps_sw_board_version = BOARD_TRIZEPS7_V1R2,
  .trizeps_board_version_str= "TRIZEPS_VII_V1R2",
  .trizeps_unique_id[0]=0,
  .trizeps_unique_id[1]=0,
};

//#define HW_OCOTP(n)  (MX6_IO_ADDRESS(OCOTP_BASE_ADDR)+(n))
#define HW_OCOTP(n)  (OCOTP_BASE_ADDR+(n))

unsigned long ddr3l            =UNDEFINED;

int trizeps7_isddr3l(void)
{
  unsigned int value, isddr3l=0;
	
  value = readl(HW_OCOTP(0x660));  // Board version;
  if( value & BOARD_TRIZEPS7_DDR3L_VALID)
    isddr3l=(value & BOARD_TRIZEPS7_HAS_DDR3L)?1:0;

  if( ddr3l == UNDEFINED )
    ddr3l = isddr3l;

  return(isddr3l);  
}

PTRIZEPS_INFO get_trizeps_board_version(void)
{
	unsigned int value;
	PTRIZEPS_INFO pTr = &TrizepsBoardVersion;
	pTr->trizeps_module            =7; 
	pTr->trizeps_sodimm            =200;
	pTr->trizeps_btwlan            =UNDEFINED;
	pTr->trizeps_extcon            =UNDEFINED; 
	pTr->trizeps_ddr3l             =UNDEFINED; 
	pTr->trizeps_hw_board_version  =BOARD_TRIZEPS7_V1R2;
	pTr->trizeps_sw_board_version  =BOARD_TRIZEPS7_V1R2;
	pTr->trizeps_board_version_str ="TRIZEPS_VII_V1R?";
	pTr->trizeps_unique_id[0]      =UNDEFINED;
	pTr->trizeps_unique_id[1]      =UNDEFINED;
        pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(1, 6);

	if(cpu_is_mx6q())               // Default for mx6q -> No DDR3L
	{  
	  pTr->trizeps_ddr3l = 0;
	}else
	{  
	  pTr->trizeps_ddr3l = 1;
	}
	
	// Keith swap  5...0 -> 0....5
	value = readl(HW_OCOTP(0x660));  // Board version;
	if( value & BOARD_TRIZEPS7_DDR3L_VALID)
	  pTr->trizeps_ddr3l=(value & BOARD_TRIZEPS7_HAS_DDR3L)?1:0;

	if( ddr3l == UNDEFINED )
	  ddr3l = pTr->trizeps_ddr3l;	

	// lowest 8 Bits for compatibility 
	switch (value&0xff)
	{
	default:
	case 0:
	  pTr->trizeps_board_version_str ="TRIZEPS_VII_V1R?";
	  pTr->trizeps_sw_board_version=BOARD_TRIZEPS7_V1R0;
	  pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(1, 6);
	  break;
	case 1:
	  pTr->trizeps_sw_board_version =BOARD_TRIZEPS7_V1R1;
	  pTr->trizeps_board_version_str="TRIZEPS_VII_V1R1";
	  pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(1, 6);
	  break;
	case 2:
	  pTr->trizeps_sw_board_version =BOARD_TRIZEPS7_V1R2;
	  pTr->trizeps_board_version_str="TRIZEPS_VII_V1R2";
	  pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(1, 6);
	  break;
	case 3:
	  pTr->trizeps_sw_board_version =BOARD_TRIZEPS7_V1R3;
	  pTr->trizeps_board_version_str="TRIZEPS_VII_V1R3";
	  pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(7, 12);
	  break;
	}
	// next 8 Bits for minor subreleases
	pTr->trizeps_hw_board_version=pTr->trizeps_sw_board_version+((value&0xff00)>>8);
	pTr->trizeps_unique_id[0]=readl(HW_OCOTP(0x410));
	pTr->trizeps_unique_id[1]=readl(HW_OCOTP(0x420));

	return(pTr);
}

char cpu_namestr[12];
char cpu_revstr[12];
char cpu_freq[10];
char cpu_grade[64];
char cpu_boottemperature[12];
char cpu_resetcause[64];

static char *get_reset_cause(void)
{
	switch (get_imx_reset_cause()) {
	case 0x00001:
	case 0x00011:
		return "POR";
	case 0x00004:
		return "CSU";
	case 0x00008:
		return "IPP USER";
	case 0x00010:
#ifdef	CONFIG_MX7
		return "WDOG1";
#else
		return "WDOG";
#endif
	case 0x00020:
		return "JTAG HIGH-Z";
	case 0x00040:
		return "JTAG SW";
	case 0x00080:
		return "WDOG3";
#ifdef CONFIG_MX7
	case 0x00100:
		return "WDOG4";
	case 0x00200:
		return "TEMPSENSE";
#elif defined(CONFIG_IMX8M)
	case 0x00100:
		return "WDOG2";
	case 0x00200:
		return "TEMPSENSE";
#else
	case 0x00100:
		return "TEMPSENSE";
	case 0x10000:
		return "WARM BOOT";
#endif
	default:
		return "unknown reset";
	}
}

int fill_cpu_env(char *cpu_namestr,char *cpu_revstr,char *cpu_freq,
		 char *cpu_grade, char *cpu_boottemperature, char *cpu_resetcause)
{
	u32 cpurev;
	__maybe_unused u32 max_freq;
	struct udevice *thermal_dev;
	int cpu_tmp, minc, maxc, ret;
	cpurev = get_cpu_rev();
#if defined(CONFIG_IMX_THERMAL)
	max_freq = get_cpu_speed_grade_hz();
	sprintf(cpu_namestr, "i.MX%s",  get_imx_type((cpurev & 0xFF000) >> 12));
	sprintf(cpu_revstr,  "rev%d.%d",(cpurev & 0x000F0) >> 4,(cpurev & 0x0000F) >> 0);	
	sprintf(cpu_freq,     "%d"     ,mxc_get_clock(MXC_ARM_CLK));

	puts("CPU:   ");
	switch (get_cpu_temp_grade(&minc, &maxc)) {
	case TEMP_AUTOMOTIVE:
		sprintf(cpu_grade,"Automotive %d-%d", minc, maxc);
		break;
	case TEMP_INDUSTRIAL:
		sprintf(cpu_grade,"Industrial %d-%d", minc, maxc);
		break;
	case TEMP_EXTCOMMERCIAL:
		sprintf(cpu_grade,"Ext. Commercial %d-%d", minc, maxc);
		break;
	default:
		sprintf(cpu_grade,"Commercial %d-%d", minc, maxc);
		break;
	}
	ret = uclass_get_device(UCLASS_THERMAL, 0, &thermal_dev);
	if (!ret) {
	  ret = thermal_get_temp(thermal_dev, &cpu_tmp);
	  if (!ret)
	      sprintf(cpu_boottemperature,"%dC", cpu_tmp);
	    else	    
	      sprintf(cpu_boottemperature,"n.a.");		    
	}
#endif
    sprintf(cpu_resetcause, "%s", get_reset_cause());
	return(0);	
}

void SetTrizepsEnvironment(void)
{
        PTRIZEPS_INFO pTr= &TrizepsBoardVersion;
	char *fdtfile;
	
	printf("SetTrizepsEnvironment\n");
	
	if(cpu_is_mx6q())               // Default for mx6q -> No DDR3L
	{  
	  env_set("cpu_type", "iMX6DQ");
	  env_set("cpu_quadext",   "-mx6q");	  
	}else
	{  
	  env_set("cpu_type", "iMX6SDL");
	  env_set("cpu_quadext",     "");	  
	}
	env_set("cpu_module",               "7");
	env_set("cpu_module_ddr_type",     ((pTr->trizeps_ddr3l!=0)?"ddr3l":"ddr3"));
	env_set("cpu_module_hw_version",   ((pTr->trizeps_ddr3l!=0)?"ddr3l":"ddr3"));
	env_set("cpu_module_versionstring", pTr->trizeps_board_version_str);

	fill_cpu_env(cpu_namestr,cpu_revstr,          cpu_freq,
		     cpu_grade,  cpu_boottemperature, cpu_resetcause);	

//	printf("SetTrizepsEnvironment CPU:%s,%s,%s,%s,%s,%s\n", cpu_namestr, cpu_revstr, cpu_freq,
//	                                                         cpu_grade, cpu_boottemperature,cpu_resetcause);

	env_set("cpu_namestr",              cpu_namestr);
	env_set("cpu_revstr" ,              cpu_revstr);
	env_set("cpu_maxfreq" ,             cpu_freq);
	env_set("cpu_grade" ,               cpu_grade);
	env_set("cpu_temperature" ,         cpu_boottemperature);
	env_set("cpu_resetcause",           cpu_resetcause);

	if (!(fdtfile =env_get("fdt_file")))
	{ 
	  if(is_mx6dq())
	    env_set("fdt_file", "oftree-mx6q.dtb");
	  else
	    env_set("fdt_file", "oftree.dtb");
	}

    env_set("board_name", "Trizeps7");
    if (is_mx6dq()) {
        env_set("board_rev", "MX6Q");
    }
    else {
        env_set("board_rev", "MX6DL");
    }

    if(gd->ram_size == 0x20000000) {
        env_set("board_mem_size", "512MiB");
    }
    else if(gd->ram_size == 0x40000000) {
        env_set("board_mem_size", "1GiB");
    }
    else if(gd->ram_size == 0x80000000) {
        env_set("board_mem_size", "2GiB");
    }

}


void PrintTrizepsInfo(void)
{
     PTRIZEPS_INFO pTr= &TrizepsBoardVersion;
     printf( KERN_ERR "********************* Trizeps7_Info *********************************\n");
     printf("Trizeps_module   Version: %lu\n",         pTr->trizeps_module);
     printf("Trizeps Sodimm   Pins:    %lu\n",         pTr->trizeps_sodimm);
     printf("Trizeps DDR      Type:    %s",        ((pTr->trizeps_ddr3l!=0)?"ddr3l":"ddr3"));
     if( ddr3l != pTr->trizeps_ddr3l )
       printf("overwrite cmdline with: %s", ((pTr->trizeps_ddr3l!=0)?"ddr3l":"ddr3"));
     else
       printf("\n");     
     printf("Trizeps Version  String:  %s\n",          pTr->trizeps_board_version_str);
     printf("Trizeps HW Board Version: 0x%lx\n",       pTr->trizeps_hw_board_version);
     printf("Trizeps SW Board Version: 0x%lx\n",       pTr->trizeps_sw_board_version);
     printf("Trizeps nReset via Gpio   %ld.%d\n",      pTr->trizeps_resetout_gpio/32+1,
	                                               (unsigned int)pTr->trizeps_resetout_gpio%32);
     printf("Trizeps UniqueID:         0x%lx-%lx\n",   pTr->trizeps_unique_id[0],
	                                               pTr->trizeps_unique_id[1]);
     printk( KERN_ERR "*********************************************************************\n");

}

int TrizepsVersionFromTo(int from,int to)
{
  if( (TrizepsBoardVersion.trizeps_sw_board_version >= from) &&
      (TrizepsBoardVersion.trizeps_sw_board_version <= to  ) )
    return(1);
  else
    return(0);
}
