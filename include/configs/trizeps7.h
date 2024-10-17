/* 
 * trizeps 7 board header file
 */

#ifndef __TRIZEPS7_CONFIG_H
#define __TRIZEPS7_CONFIG_H

#include "mx6_common.h"

#define CONFIG_MXC_UART_BASE        UART2_BASE

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR   USDHC4_BASE_ADDR
#define CONFIG_SYS_FSL_USDHC_NUM    3
#define CONFIG_SYS_MMC_ENV_DEV      0 /* SDHC4 */

#define CONFIG_SYS_MAX_FLASH_BANKS  1
#define CONFIG_SYS_MALLOC_LEN       (10 * SZ_1M)
#define CONFIG_SYS_SDRAM_BASE       MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_ADDR    IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE    IRAM_SIZE
#define CONFIG_SYS_INIT_SP_OFFSET   (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR     (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#ifndef MMC_BOOTLOADER_PARTITION
#define MMC_BOOTLOADER_PARTITION  "0"
#endif

#define CONSOLE_DEV               "ttymxc1"
#define CONFIG_LOADADDR             0x12000000
#define MMCROOT   	"/dev/mmcblk3p2"    /* Sigma init in intramfs parses this to mmcblk0p3 */
#define MMCROOT_2	"/dev/mmcblk0p3"
#define SUB_LOADBOOTSCRIPT ""
#define BOOTENV ""
#define EMMC_ENV ""

/* I2C Configs */
#define ENABLE_I2C_MXC          1
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1 /* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2 /* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED    100000

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX       2
#define CONFIG_BAUDRATE         115200

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_XCV_TYPE     RMII
#define IMX_FEC_BASE            ENET_BASE_ADDR
#define CONFIG_PHY_RESET_DELAY  1000
#define CONFIG_RMII             1

#define CONFIG_ETHPRIME         "FEC"
#define CONFIG_FEC_MXC_PHYADDR  1

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

#define CONFIG_IPADDR           10.199.0.155   /* 192.168.1.103	*/
#define CONFIG_SERVERIP         10.199.0.78    /* 192.168.1.101 */
#define CONFIG_NETMASK          255.255.255.0    /* 255.255.255.0 */

#define CONFIG_IMX_THERMAL

/* Framebuffer */
// #define CONFIG_VIDEO
#define CONFIG_DM_VIDEO
#define CONFIG_VIDEO_PCI_DEFAULT_FB_SIZE 0x0
//#define CONFIG_VIDEO_IPUV3
// #define CONFIG_CFB_CONSOLE

// #define CONFIG_VGA_AS_SINGLE_DEVICE
// #define CONFIG_SYS_CONSOLE_IS_IN_ENV
// #define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
// #define CONFIG_VIDEO_BMP_RLE8

// #define CONFIG_SPLASH_SCREEN
// #define CONFIG_SPLASH_SCREEN_ALIGN
// #define CONFIG_BMP_16BPP
// #define CONFIG_VIDEO_LOGO
// #define CONFIG_VIDEO_BMP_LOGO
// #define CONFIG_IPUV3_CLK 260000000
// #define CONFIG_IMX_HDMI
// #define CONFIG_IMX_VIDEO_SKIP
// #define CONFIG_CONSOLE_MUX 

#define UBOOT_IMAGE      zImage /* orig: zImage */


#if 1
#define CONFIG_BOOTCOMMAND                             \
	 "if test ${usbboot} = yes; then "                 \
	   "echo Starting USB ; "                          \
	   "usb start; "			                       \
	   "if run loadbootscriptfatusb; then "	           \
	     "echo running bootscript from usbfat; "       \
	     "run bootscript; "                            \
           "fi; "				                       \
          "fi; "				                       \
	  "mmc dev ${mmcdev};"                             \
	  "if mmc rescan; then "                           \
	    "if run loadbootscriptext4; then "             \
		"echo running bootscript from ext4; "          \
		"run bootscript; "                             \
	    "else "                                        \
	       "if run loadbootscriptfat; then "           \
		  "echo running bootscript from fat; "         \
		  "run bootscript; "                           \
	       "else "                                     \
	        "if run loadbootscriptfatandroid; then "   \
		   "echo running bootscript from fat; "        \
		   "run bootscript; "                          \
	        "else "                                    \
		  "if run loadimageext4; then "                \
		      "echo loaded image from ext4;"           \
		      "run mmcboot; "                          \
		  "else "                                      \
			"if run loadimagefat; then "               \
		          "echo loaded image from fat;"        \
			  "run mmcboot; "                          \
			"else "                                    \
                           "mw.b $fdt_addr 0 0x40;"    \
                           "run loadfdtandroid;"       \
	                   "if  run loadandroid; then"     \
	                       ";"                         \
	                   "else "                         \
                             "run netboot; "           \
	                   "fi; "                          \
			"fi; "                                     \
		  "fi; "                                       \
	        "fi; "                                     \
	      "fi; "                                       \
	  "fi; "                                           \
	"else run netboot; fi; " 
#else
/*
#define CONFIG_BOOTCOMMAND                             \
    "if run loadbootscriptext2; then "                 \
        "echo running SuB bootscript from ext2; "      \
    "run bootscript; "                                 \
    "else "                                            \
        "run netboot;                                  \
	fi;"
#endif
*/
#define CONFIG_BOOTCOMMAND                             \
    "echo minimal bootcommand; "                       \
    "mmc dev ${mmcdev};"                               \
    "if run loadbootscriptext4; then "                 \
        "echo running bootscript from ext4; "          \
        "run bootscript; "                             \
    "else "                                            \
        "echo boot attempt failed; "                   \
    "fi; "
#endif

#if 1
#define CONFIG_EXTRA_ENV_SETTINGS   \
        "skip_logo=0\0"             \
        "bmp_addr=0x1FF48000\0"     \
        "bmp_file=bootlogo.bmp\0"   \
	"script=boot.scr\0"         \
        "script_addr=0x1FF58000\0"  \
	"image=" __stringify(UBOOT_IMAGE) "\0"            \
	"fdt_addr=0x1FF60000\0"     \
	"fdt_high=0xffffffff\0"	    \
	"boot_fdt=try\0"            \
        "mmc_bootloader_partition=" MMC_BOOTLOADER_PARTITION "\0" \
	"ip_dyn=yes\0"              \
	"console=" CONSOLE_DEV "\0" \
	"dfuspi=dfu 0 sf 0:0:10000000:0\0" \
	"dfu_alt_info_spl=spl raw 0x400\0" \
	"dfu_alt_info_img=u-boot raw 0x10000\0" \
	"dfu_alt_info=spl raw 0x400\0" \
	"initrd_high=0xffffffff\0" \
        "usbboot=0\0" \
        "usbdev=0\0"  \
        "usbpart=1\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcpartext4=1\0" \
	"partfdtandroid=8\0" \
	"mmcroot="  MMCROOT   " ro\0" \
	"mmcroot2=" MMCROOT_2 " rootwait rw\0" \
        "fuse_sd=0 5 0x3840 0x10\0" \
        "fuse_mmc=0 5 0x3860 0x10\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	EMMC_ENV  \
        "initrd= - \0"                                                                              \
        "bootextra=\0"                                                                              \
	"mmcargs=setenv bootargs console=${console},${baudrate} ${bootextra} root=${mmcroot}\0"     \
	"loadbootscriptext4=ext4load mmc ${mmcdev}:${mmcpartext4} ${script_addr} ${script};\0"      \
	"loadbootscriptfat=fatload mmc ${mmcdev}:${mmcpart} ${script_addr} ${script};\0"            \
	"loadbootscriptandroid=fatload mmc ${mmcdev}:${partfdtandroid} ${script_addr} ${script};\0" \
	"loadbootscriptfatusb=fatload usb ${usbdev}:${usbpart} ${script_addr} ${script};\0"         \
	"bootscript=echo Running bootscript...; source ${script_addr}\0"                            \
	"loadimageext4=ext4load mmc ${mmcdev}:${mmcpartext4} ${loadaddr} ${image}\0"                \
	"loadimagefat=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0"                      \
	"loadandroid=boota  mmc${mmcdev}\0"                                                         \
	"exportsd=ums 0 mmc ${mmcdev}\0"                                                            \
	"burnfuses4sd=fuse prog ${fuse_sd}\0"                                                       \
	"burnfuses4mmc=fuse prog ${fuse_mmc}\0"                                                     \
	"brickme_sd=mmc erase 2 10\0"                                                               \
        "brickme_mmc=mmc partconf 0 1 0 1 ; mmc erase 2 10 ;mmc partconf 0 1 1 1 ; mmc erase 2 10 ;mmc partconf 0 0 0 0 ; mmc erase 2 10 \0" \
	"initialdeploymentsd=run brickme_sd ; fuse prog -y ${fuse_sd}; ums 0 ${mmcdev} \0"      \
	"initialdeploymmcuserpart=echo <Brickmemmc>; run brickme_mmc ; fuse prog -y ${fuse_mmc} ; ums 0 ${mmcdev} \0" \
	"initialdeploymmcbootpart=echo <Brickmemmc>; run brickme_mmc ; fuse prog -y ${fuse_mmc} ; ums 0 ${mmcdev} \0" \
	"bootcmd_android_recovery=echo Booting Recovery Image from mmc${mmcdev}..; "                \
              "mw.b $fdt_addr 0 0x40;"                                                              \
              "run loadfdtandroid;"                                                                 \
              "boota  mmc${mmcdev} recovery\0"                                                      \
	"loaddebian=boota  mmc${mmcdev}\0"                                                          \
	"loadbmpandroid=fatload mmc ${mmcdev}:${partfdtandroid} ${bmp_addr} ${bmp_file}\0"          \
	"loadbmpext4=ext4load mmc ${mmcdev}:${mmcpartext4} ${bmp_addr} ${bmp_file}\0"               \
        "clearfdtspace=mw.b $fdt_addr 0 0x40"                                                       \
	"loadfdtfat=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0"                     \
	"loadfdtext4=ext4load mmc ${mmcdev}:${mmcpartext4} ${fdt_addr} ${fdt_file}\0"               \
	"loadfdtandroid=fatload mmc ${mmcdev}:${partfdtandroid} ${fdt_addr} ${fdt_file}\0"          \
	"loadfdt=echo loading fdt..;"                                                               \
            "if run loadfdtext4; then echo loadtdext4 ok; else echo trying from fat....; "          \
            "if run loadfdtfat;  then echo loadftdfat ok; fi; fi;\0"                                \
	"mmcboot=echo Booting from mmc ...; "                                                       \
		"run mmcargs; "                                                                     \
                "run loadbmpext4; "                                                                 \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then "                        \
			"if run loadfdt; then "                                                     \
				"bootz ${loadaddr} ${initrd} ${fdt_addr}; "                         \
			"else "                                                                     \
				"if test ${boot_fdt} = try; then "                                  \
					"bootz; "                                                   \
				"else "                                                             \
					"echo WARN: Cannot load the DT; "                           \
				"fi; "                                                              \
			"fi; "                                                                      \
		"else "                                                                             \
			"bootz; "                                                                   \
		"fi;\0"                                                                             \
	"netargs=setenv bootargs console=${console},${baudrate} "                                   \
		"root=/dev/nfs "                                                                    \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,\0"                                      \
	"netboot=echo Booting from net ...; "                                                       \
		"run netargs; "                                                                     \
		"if dhcp ${loadaddr} ${image}; then "                                               \
                    "if tftp ${fdt_addr} ${fdt_file}; then "                                        \
		        "bootz ${loadaddr} ${initrd} ${fdt_addr}; "                                 \
		    "else "                                                                         \
	                "echo WARN: Cannot load the DT; "                                           \
		    "fi; "                                                                          \
		"else "                                                                             \
                    "echo WARN: Cannot load OS-Image; "                                             \
		"fi;\0"                                                                             \
        "testusb=echo TESTING usb.....;"                                                            \
                "usb start;\0"                                                                      \
        "findfdt="	\
	 "if test $fdt_file = oftree.dtb; then " \
	   "if test $board_name=ipan7 && test $board_rev=MX6Q;  then setenv fdt_file ipan7-mx6q_v1r3.dtb;  fi; " \
	   "if test $board_name=ipan7 && test $board_rev=MX6DL; then setenv fdt_file ipan7-mx6dl_v1r3.dtb; fi; " \
	   "if test $board_name=ipan5 && test $board_rev=MX6Q;  then setenv fdt_file ipan5-mx6q_v1r3.dtb;  fi; " \
	   "if test $board_name=ipan5 && test $board_rev=MX6DL; then setenv fdt_file ipan5-mx6dl_v1r3.dtb; fi; " \
	   "if test $fdt_file=oftree.dtb; then echo Could not determine dtb file, trying oftree.dtb;       fi; " \
	 "else echo no dtb scan; fi;\0" \
	 SUB_LOADBOOTSCRIPT BOOTENV

#else 
#define CONFIG_EXTRA_ENV_SETTINGS \
    "script=boot.scr\0"         \
    "script_addr=0x1FF58000\0"  \
	"image=zImage\0"            \
    "fdt_file=oftree.dtb\0"     \
	"fdt_addr=0x1FF60000\0"     \
	"fdt_high=0xffffffff\0"	    \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"console=" CONSOLE_DEV "\0" \
	"mmcroot="  MMCROOT   " ro\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} ${bootextra} root=${mmcroot}\0"     \
	"loadbootscriptext4=ext4load mmc ${mmcdev}:${mmcpartext4} ${script_addr} ${script};\0" \
    "bootscript=echo Running bootscript...; source ${script_addr}\0"               

#endif


#endif /* __TRIZEPS7_CONFIG_H */

