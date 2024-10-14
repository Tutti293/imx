/*
 ;------------------------------------------------------------------------------
 ;
 ;  Copyright (C) 2014, Keith & Koep GmbH
 ;  THIS SOURCE CODE, AND ITS USE AND DISTRIBUTION, IS SUBJECT TO THE TERMS
 ;  AND CONDITIONS OF THE APPLICABLE LICENSE AGREEMENT
 ;
 ;------------------------------------------------------------------------------
 ;
 ;   FILE:   ivt_init.s
 ;   
 ;   First instruction to be executed is this jump instruction, control 
 ;   is passed to the entry point of eboot from this function
 ;   This object file should be located at first address of Storage.
 ;------------------------------------------------------------------------------
*/
#include <config.h>
//#include <asm/arch/mx6.h>
#include "include/mx6.h"
#define TEXT_BASE                            CONFIG_SYS_TEXT_BASE	
#if 0
#include <asm/system.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
/#include <linux/linkage.h>
#endif
#define CONFIG_FLASH_HEADER                  1
#define CONFIG_FLASH_HEADER_OFFSET           0x400
	
#define CSP_BASE_MEM_PA_DRAM_LOW             0x10000000
#define CSP_BASE_MEM_PA_DRAM_HIGH            0x80000000
#define CSP_BASE_MEM_PA_ROM                  0x00000000 	
#define IMAGE_BOOT_RAMDEV_RAM_PA_START       CSP_BASE_MEM_PA_DRAM_LOW
#define IMAGE_BOOT_RAMDEV_RAM_UA_START       0xA0000000
#define IMAGE_BOOT_RAMDEV_RAM_CA_START       0x80000000

#define IMAGE_SHARE_EVT_RAM_OFFSET          (0)
#define IMAGE_SHARE_EVT_RAM_SIZE            (4*1024)  	
#define IMAGE_SHARE_ARGS_RAM_OFFSET         (IMAGE_SHARE_EVT_RAM_OFFSET+IMAGE_SHARE_EVT_RAM_SIZE)
#define IMAGE_SHARE_ARGS_RAM_SIZE           (4*1024) 	
#define IMAGE_BOOT_STACK_RAM_OFFSET         (IMAGE_SHARE_ARGS_RAM_OFFSET+IMAGE_SHARE_ARGS_RAM_SIZE) 
#define IMAGE_BOOT_STACK_RAM_SIZE           (232*1024)
#define IMAGE_BOOT_BOOTPT_RAM_OFFSET        (IMAGE_BOOT_STACK_RAM_OFFSET+IMAGE_BOOT_STACK_RAM_SIZE)
#define IMAGE_BOOT_BOOTPT_RAM_SIZE          (16*1024)

#define IMAGE_BOOT_BOOTIMAGE_SD_SIZE        (512*1024)
#define CSP_BASE_REG_PA_CCM                 (0x020C4000)
#define CSP_BASE_REG_PA_IOMUXC              (0x020E0000)
#define CSP_BASE_REG_PA_MMDC0               (0x021B0000)
	
#define IMAGE_BOOT_BOOTPT_RAM_OFFSET        (IMAGE_BOOT_STACK_RAM_OFFSET+IMAGE_BOOT_STACK_RAM_SIZE) 	
#define IMAGE_BOOT_BOOTIMAGE_RAM_OFFSET     (IMAGE_BOOT_BOOTPT_RAM_OFFSET+IMAGE_BOOT_BOOTPT_RAM_SIZE) 	
#define IMAGE_BOOT_BOOTIMAGE_RAM_PA_START   (IMAGE_BOOT_RAMDEV_RAM_PA_START+IMAGE_BOOT_BOOTIMAGE_RAM_OFFSET) 

#ifdef	CONFIG_FLASH_HEADER
#ifndef CONFIG_FLASH_HEADER_OFFSET
# error "Must define the offset of flash header"
#endif
#endif
	
#define IRAM_FREE_SPACE_START   0x00907000     /*  Free space, IRAM_START + 7 * 4K */
#define NFC_BUFFER_ADDR         0xF7FF0000     /*  TODO:  */
#define IMAGE_HDR_SIZE          0x1000         /*  Reserve 4K for plugin code */
#define BASE_ADDR_OFFSET_4K     0x1000
#define BASE_ADDR_OFFSET_1K     0x0400
#define BASE_ADDR_OFFSET_256    0x0100
#define BIN_OFFSET              0       /* 0x1000  This offset is from bin file generation by romimage. */
                                        /* We cut 4k when programming the image into sd card so  */
                                        /* that we rom code can get correct ivt, when move image */
                                        /* from storage to ram, we must add it back, so that all */
	                                /* address in code is correct. */

/* */
/*  ROM constants*/
/* */
#define ROM_VER_OFFSET          0x0048

/* */
/*  CCM constants*/
/* */
#define CCM_CBCDR_OFFSET        0x0014

/* */
/*  Board ID constant*/
/* */
/* IIM_BOARDID_HIGH_OFFSET EQU     0x0878*/
/* IIM_BOARDID_LOW_OFFSET  EQU     0x087C*/

/* BOARD_REV_ARD_REVA      EQU     0x1*/
/* BOARD_REV_ARD_REVB      EQU     0x2*/

//#define ANDROID_SETTINGS           0x01
#define DDR_AUTO_POWERDOWN         0x01

/*  Image destination address is the ram address which is aligned with address 0 of */
/*  corresponding boot device. */

#define IMGSDMMC                     1
	
#if IMGSDMMC
/*  Config for SD/MMC Boot*/
 #define IVT_OFFSET              BASE_ADDR_OFFSET_1K
 #define BOOT_DATA_IMG_LEN       IMAGE_BOOT_BOOTIMAGE_SD_SIZE+IVT_OFFSET  /*  512K + OFFSET*/
 #else
  #if  IMGATA
     /* Config for ATA Boot*/
#define IVT_OFFSET              BASE_ADDR_OFFSET_1K
#define BOOT_DATA_IMG_LEN       IMAGE_BOOT_BOOTIMAGE_ATA_SIZE+IVT_OFFSET
  #else	
   #if IMGNAND
     /* Config for NAND Boot*/
#define IVT_OFFSET              BASE_ADDR_OFFSET_1K
#define BOOT_DATA_IMG_LEN       IMAGE_BOOT_BOOTIMAGE_RAM_SIZE+IVT_OFFSET
   #else
    /* Config for SPI Boot*/
#define IVT_OFFSET              BASE_ADDR_OFFSET_1K
#define BOOT_DATA_IMG_LEN       IMAGE_BOOT_BOOTIMAGE_SD_SIZE + IVT_OFFSET/* IMAGE_BOOT_BOOTIMAGE_SPI_SIZE+IVT_OFFSET*/
   #endif
  #endif
 #endif

#define IVT_ADDRESS              IMAGE_BOOT_BOOTIMAGE_RAM_PA_START + BIN_OFFSET   

	.global _FlashHeader
	.global _FlashHeaderEnd
	.ALIGN 4
	.section ".ivt", "x"	
_FlashHeader:	
    b _start	
    /*  IVT (Image Vector Table) format*/
    /* */
    /*    0x00    HEADER (tag, length, version)*/
    /*    0x04    ENTRY ADDRESS*/
    /*    0x08    RESERVED1*/
    /*    0x0C    DCD ADDRESS*/
    /*    0x10    BOOT DATA ADDRESS*/
    /*    0X14    SELF ADDRESS*/
    /*    0X18    CSF ADDRESS*/
    /*    0X1C    RESERVED2*/
    .word IVT_ADDR	
    .word IRAM_FREE_SPACE_START
    .word IVT_OFFSET
    .word PLUGIN_ENTRY
    .word IVT_ADDR
    .org  CONFIG_FLASH_HEADER_OFFSET

// #define IVT_HEADER 0xdeadbeef
#define IVT_HEADER 0x402000D1     /* 0xD1002040 */
	
IVT_ADDR:
    .word IVT_HEADER                                                  /*  0x00  HEADER (tag, length, version)*/	
    .word IRAM_FREE_SPACE_START+IVT_OFFSET+(PLUGIN_ENTRY-IVT_ADDR)    /*  0x04  ENTRY ADDRESS*/
    .word 0                                                           /*  0x08  RESERVED1*/
    .word 0                                                           /*  0x0C  DCD ADDRESS*/
    .word IRAM_FREE_SPACE_START+IVT_OFFSET+(BOOT_DATA_ADDR-IVT_ADDR)  /*   0x10  BOOT DATA ADDRESS*/
    .word IRAM_FREE_SPACE_START+IVT_OFFSET                            /*  0x14  SELF ADDRESS*/
    .word 0                                                           /*  0x18  CSF ADDRESS*/
    .word 0                                                           /*  0x1C  RESERVED2*/


/*  Second IVT to give entry point into the bootloader copied to DDR */
IVT2_ADDR:	
    .word IVT_HEADER                                                  /*  HEADER*/
    .word _start /* IMAGE_BOOT_BOOTIMAGE_RAM_PA_START+BIN_OFFSET+IMAGE_HDR_SIZE   IMAGE START ADDRESS FOR EXECUTE*/
    .word 0                                                           /*  RESERVED1*/
    .word 0                                                           /*  .word ADDRESS*/
    .word BOOT_DATA2_ADDR /* IRAM_FREE_SPACE_START+IVT_OFFSET+(BOOT_DATA2_ADDR-IVT_ADDR)  BOOT DATA ADDRESS*/
    .word IVT2_ADDR                                                   /*  SELF ADDRESS*/
    .word 0                                                           /*  CSF ADDRESS*/
    .word 0                                                           /*  RESERVED*/

    /*  IVT boot data format*/
    /*    0x00    IMAGE START ADDR*/
    /*    0x04    IMAGE SIZE*/
    /*    0x08    PLUGIN FLAG    */
BOOT_DATA_ADDR:	
boot_data:
    .word IRAM_FREE_SPACE_START                                       /*  PLUGIN DESTINTATION ADDRESS */
image_len:
    .word IMAGE_HDR_SIZE              /* _end_of_copy-TEXT_BASE+CONFIG_FLASH_HEADER_OFFSET */
    .word 1                           /*  PLUGIN FLAG*/

DDR_DEST_ADDR:	.word  TEXT_BASE      /* (IVT_ADDRESS - IVT_OFFSET) */
COPY_SIZE:	.word  _end_of_copy  - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET /*  BOOT_DATA_IMG_LEN */

    /*  IVT boot data format*/
    /*    0x00    IMAGE START ADDR*/
    /*    0x04    IMAGE SIZE*/
    /*    0x08    PLUGIN FLAG    */
BOOT_DATA2_ADDR:	
    .word TEXT_BASE                   /*  START OF DESTINTATION ADDRESS*/
    .word _end_of_copy  - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET     /*  IMAGE SIZE*/
    .word 0                           /*  PLUGIN FLAG*/

    /*  ROM uses pu_irom_run_plugin to execute plugin.  The plugin is called*/
    /*  as a function pointer with the following prototype*/
    /*    BOOL *(plugin_f)(void **start, size_t *bytes, UINT32 *ivt_offset)*/

//	.global PLUGIN_ENTRY //FIXME
PLUGIN_ENTRY:
    /*  Save the return address and plugin function parameters */
    /*  (saved on ROM code stack)*/
    /* */
    /*    r0 = **start*/
    /*    r1 = *bytes*/
    /*    r2 = *ivt_offset*/
    /* */

    /*  enters at 0x907460*/
    /*  starts at offset 0x60 in eboot.nb0*/
    /*  processor is in arm-encoded instruction mode*/

    /*  PUSHSTACK2LR_ORIG R0, R7 */
    stmfd   sp!,{r0-r7,lr}
    /* We should distinguish USB recovery mode(SDP mode) and internal boot mode.*/
    /* If ROM runs in SDP mode, then it needn't load boot code from storage media.*/
    /* If ROM runs in SDP mode, then r0 must be 0x00*/
    /* If ROM runs in internal boot mode, then r0 should be larger than IRAM base address.*/

    mov     r7, r0
    cmp     r0, #0x00900000   /*  check if in iram        00900000 - 0093FFFF 256K IRAM*/

    ldr     r3, DDR_DEST_ADDR
    strhi r3, [r0]              /*  STR if HI*/
    ldr     r3, =BOOT_DATA_IMG_LEN
    strhi r3, [r1]
    ldr     r3, =(IVT_OFFSET+(IVT2_ADDR-IVT_ADDR))
    strhi r3, [r2]              /*  change content in location pointed by r0~r2*/

    /*  Save the necessary address.*/
    adr     r4, DDR_DEST_ADDR
    adr     r5, COPY_SIZE
    adr     r6, BOOT_DATA2_ADDR

    /*  Check ROM ID to determine silicon revision*/
    /* ldr     r0, =CSP_BASE_MEM_PA_ROM*/
    /* ldr     r3, [r0, #ROM_VER_OFFSET]*/
    /* cmp     r3, #0x20               ;check if it's TO1 silicon*/
			
/* //=============================================================================			*/
/* // Disable	WDOG		*/
/* //=============================================================================			*/
    /*  Disable WDOG*/
    /*  setmem /16 0x020bc000 = 0x30*/
    /* ldr     r0, =CSP_BASE_REG_PA_WDOG1*/
    /* ldr     r1, =0x30*/
    /* ldr     r2, =0x0*/
    /* str     r1, [r0, r2]*/
			
/* //=============================================================================			*/
/* // ; Configure CCM //  Enable all clocks (they are disabled by ROM code)			*/
/* //=============================================================================	*/
	ldr     r0, =CSP_BASE_REG_PA_CCM
		
    /*  setmem /32 0x020c4068 = 0xffffffff*/
    ldr     r1, =0xffffffff
    ldr     r2, =0x068
    str     r1, [r0, r2]

    /* setmem /32	0x020c406c =	0xffffffff	*/
    ldr     r1, =0xffffffff
    ldr     r2, =0x06c
    str     r1, [r0, r2]

    /* setmem /32	0x020c4070 =	0xffffffff*/
    ldr     r1, =0xffffffff
    ldr     r2, =0x070
    str     r1, [r0, r2]

    /* setmem /32	0x020c4074 =	0xffffffff	*/
    ldr     r1, =0xffffffff
    ldr     r2, =0x074
    str     r1, [r0, r2]

    /* setmem /32	0x020c4078 =	0xffffffff	*/
    ldr     r1, =0xffffffff
    ldr     r2, =0x078
    str     r1, [r0, r2]

    /* setmem /32	0x020c407c =	0xffffffff	*/
    ldr     r1, =0xffffffff
    ldr     r2, =0x07c
    str     r1, [r0, r2]

    /* setmem /32	0x020c4080 =	0xffffffff	*/
    ldr     r1, =0xffffffff
    ldr     r2, =0x080
    str     r1, [r0, r2]

    /*  setmem /32	0x020c4084 =	0xffffffff	*/
    ldr     r1, =0xffffffff
    ldr     r2, =0x084
    str     r1, [r0, r2]
			
/* //=============================================================================			*/
/* // IOMUX			*/
/* //=============================================================================	*/
	
	/*  Quad/Dual=0x..63...., Solo/Lite=0x..61....*/
	ldr		r1, =0x20C8260	
	ldr		r1,[r1]
	tst		r1, #0x00020000
	ldr     r0, =CSP_BASE_REG_PA_IOMUXC/*  IOMUXC_BASE_ADDR*/

/* //DDR IO TYPE:*/
	/* setmem /32	0x020e0774 =	0x000C0000*/
	ldr     r1, =0x000C0000		/* IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE */
    ldreq   r2, =0x774			/* solo*/
    ldrne	r2, =0x798			/* quad*/
    str     r1, [r0, r2]
	
	/* setmem /32	0x020e0754 =	0x00000000*/
	ldr     r1, =0x00000000		/* IOMUXC_SW_PAD_CTL_GRP_DDRPKE */
	ldreq   r2, =0x754			/* solo*/
	ldrne	r2, =0x758			/* quad*/
	str     r1, [r0, r2]
			
/* //CLOCK:			*/
	/* setmem /32	0x020e04ac =	0x00000030	*/
/* tocheck	ldr     r1, =0x00000030		;IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_0*/
	ldr		r1, =0x00020030		/* IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_0*/
	ldreq   r2, =0x4ac			/* solo*/
	ldrne	r2, =0x588			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e04b0 =	0x00000030*/
/* tocheck	ldr     r1, =0x00000030		;IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_1*/
	ldr     r1, =0x00020030		/* IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_1*/
	ldreq   r2, =0x4b0			/* solo*/
	ldrne	r2, =0x594			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32  0x020e0590 =    0x00003000  // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCKE0*/
	ldr		r1, =0x00003000		/* IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCKE0*/
	ldreq	r2, =0x4A4			/* solo*/
	ldrne	r2, =0x590			/* quad*/
	str		r1, [r0, r2]
	
	/* setmem /32  0x020e0598 =    0x00003000  // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCKE1			*/
	ldr		r1, =0x00003000		/* IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCKE1*/
	ldreq	r2, =0x4A8			/* solo*/
	ldrne	r2, =0x598			/* quad*/
	str		r1, [r0, r2]
								
/* //ADDRESS:			*/
	/* setmem /32	0x020e0464 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_CAS*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x464			/* solo*/
	ldrne	r2, =0x56c			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e0490 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_RAS*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x490			/* solo*/
	ldrne	r2, =0x578			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e074c =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_ADDDS */
	ldr     r1, =0x00000030
	ldr     r2, =0x74c			/* solo&quad*/
	str     r1, [r0, r2]
			
/* //CONTROL:			*/
	/* setmem /32	0x020e0494 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_RESET*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x494			/* solo*/
	ldrne	r2, =0x57c			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e04a0 =	0x00000000	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDBA2 - DSE can be configured using Group Control Register: IOMUXC_SW_PAD_CTL_GRP_CTLDS*/
	ldr     r1, =0x00000000
	ldreq   r2, =0x4a0			/* solo*/
	ldrne	r2, =0x58c			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e04b4 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT0*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x4b4			/* solo*/
	ldrne	r2, =0x59c			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e04b8 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT1*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x4b8			/* solo*/
	ldrne	r2, =0x5a0			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e076c =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_CTLDS */
	ldr     r1, =0x00000030
	ldreq   r2, =0x76c			/* solo*/
	ldrne	r2, =0x78c			/* quad*/
	str     r1, [r0, r2]	

	
/* //DATA STROBE:			*/
	/* setmem /32	0x020e0750 =	0x00020000	// IOMUXC_SW_PAD_CTL_GRP_DDRMODE_CTL */
	ldr     r1, =0x00020000
	ldr     r2, =0x750			/* solo&quad*/
	str     r1, [r0, r2]
			
	/* setmem /32	0x020e04bc =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x4bc			/* solo*/
	ldrne	r2, =0x5a8			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e04c0 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x4c0			/* solo*/
	ldrne	r2, =0x5b0			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e04c4 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x4c4			/* solo*/
	ldrne	r2, =0x524			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e04c8 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x4c8			/* solo*/
	ldrne	r2, =0x51c			/* quad*/
	str     r1, [r0, r2]	

	/* setmem /32	0x020e04cc =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x4cc			/* solo*/
	ldrne	r2, =0x518			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e04d0 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x4d0			/* solo*/
	ldrne	r2, =0x50c			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e04d4 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6 */
	ldr     r1, =0x00000030
	ldreq   r2, =0x4d4			/* solo*/
	ldrne	r2, =0x5b8			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e04d8 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7 */
	ldr     r1, =0x00000030
	ldreq   r2, =0x4d8			/* solo*/
	ldrne	r2, =0x5c0			/* quad*/
	str     r1, [r0, r2]

				
/* //DATA:			*/
	/* setmem /32	0x020e0760 =	0x00020000	// IOMUXC_SW_PAD_CTL_GRP_DDRMODE*/
	ldr     r1, =0x00020000
	ldreq   r2, =0x760			/* solo*/
	ldrne	r2, =0x774			/* quad*/
	str     r1, [r0, r2]		
	
	/* setmem /32	0x020e0764 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B0DS*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x764			/* solo*/
	ldrne	r2, =0x784			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e0770 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B1DS */
	ldr     r1, =0x00000030
	ldreq   r2, =0x770			/* solo*/
	ldrne	r2, =0x788			/* quad*/
	str     r1, [r0, r2]
	
	/* setmem /32	0x020e0778 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B2DS*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x778			/* solo*/
	ldrne	r2, =0x794			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e077c =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B3DS*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x77c			/* solo*/
	ldrne	r2, =0x79c			/* quad*/
	str     r1, [r0, r2]	
		
	/* setmem /32	0x020e0780 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B4DS*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x780			/* solo*/
	ldrne	r2, =0x7a0			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e0784 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B5DS*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x784			/* solo*/
	ldrne	r2, =0x7a4			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e078c =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B6DS*/
	ldr     r1, =0x00000030
	ldreq   r2, =0x78c			/* solo*/
	ldrne	r2, =0x7a8			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e0748 =	0x00000030	// IOMUXC_SW_PAD_CTL_GRP_B7DS*/
	ldr     r1, =0x00000030
	ldr	    r2, =0x748			/* solo&quad*/
	str     r1, [r0, r2]		
						
	/* setmem /32	0x020e0470 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM0*/
	ldr     r1, =0x00020030
	ldreq   r2, =0x470			/* solo*/
	ldrne	r2, =0x5ac			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e0474 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM1*/
	ldr     r1, =0x00020030
	ldreq   r2, =0x474			/* solo*/
	ldrne	r2, =0x5b4			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e0478 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM2*/
	ldr     r1, =0x00020030
	ldreq   r2, =0x478			/* solo*/
	ldrne	r2, =0x528			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e047c =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM3*/
	ldr     r1, =0x00020030
	ldreq   r2, =0x47c			/* solo*/
	ldrne	r2, =0x520			/* quad*/
	str     r1, [r0, r2]	

	/* setmem /32	0x020e0480 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM4*/
	ldr     r1, =0x00020030
	ldreq   r2, =0x480			/* solo*/
	ldrne	r2, =0x514			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e0484 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM5*/
	ldr     r1, =0x00020030
	ldreq   r2, =0x484			/* solo*/
	ldrne	r2, =0x510			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e0488 =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM6*/
	ldr     r1, =0x00020030
	ldreq   r2, =0x488			/* solo*/
	ldrne	r2, =0x5bc			/* quad*/
	str     r1, [r0, r2]	
	
	/* setmem /32	0x020e048c =	0x00000030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM7*/
	ldr     r1, =0x00020030
	ldreq   r2, =0x48c			/* solo*/
	ldrne	r2, =0x5c4			/* quad*/
	str     r1, [r0, r2]	

	
/* //=============================================================================			*/
/* // DDR Controller Registers			*/
/* //=============================================================================			*/
/* //=============================================================================			*/
	ldr     r0, =CSP_BASE_REG_PA_MMDC0/*  0x021b0000	*/
	
	/* setmem /32	0x021b0800 =	0xa1390003 	// DDR_PHY_P0_MPZQHWCTRL, enable both one-time & periodic HW ZQ calibration.*/
	ldr		r1, =0xa1390003		/* solo&quad*/
	ldr     r2, =0x800			/* solo&quad*/
	str     r1, [r0, r2]
	/* setmem /32	0x021b4800 =	0xa5380003 	// DDR_PHY_P0_MPZQHWCTRL, enable both one-time & periodic HW ZQ calibration.*/
	ldr	r1, =0xa1390003		/* quad*/
	ldr	r2,	=0x4800		/* quad*/
	str	r2, [r0, r2]		/* quad*/
			
/* // write leveling, based on Freescale board layout and T topology			*/
/* // For target board, may need to run write leveling calibration 			*/
/* // to fine tune these settings			*/
/* // If target board does not use T topology, then these registers			*/
/* // should either be cleared or write leveling calibration can be run			*/
	
	/* setmem /32	0x021b080c = 	0x001F001F	*/
	ldreq	r1, =0x001F001F		/* solo*/
	ldrne   r1, =0x000C001C		/* quad*/
	ldr     r2, =0x80C
	str     r1, [r0, r2]	
	
	/* setmem /32	0x021b0810 = 	0x001F001F	*/
	ldreq	r1, =0x001F001F		/* solo*/
	ldrne   r1, =0x001E0011		/* quad*/
	ldr     r2, =0x810
	str     r1, [r0, r2]	

	/* setmem /32	0x021b480c = 	0x00440044	*/
	ldreq	r1, =0x001F001F		/* solo*/
	ldrne   r1, =0x00100023		/* quad*/
	ldr     r2, =0x480c
	str     r1, [r0, r2]	
	
	/* setmem /32	0x021b4810 = 	0x00440044	*/
	ldreq	r1, =0x001F001F		/* solo*/
	ldrne   r1, =0x00040013	/* quad*/
	ldr     r2, =0x4810
	str     r1, [r0, r2]	
				
/* //######################################################			*/
/* //calibration values based on calibration compare of 0x00ffff00:			*/
/* //Note, these calibration values are based on Freescale's board			*/
/* //May need to run calibration on target board to fine tune these 			*/
/* //######################################################			*/
			
/* //Read DQS Gating calibration			*/
	/* setmem /32	0x021b083c =	0x42190219	// MPDGCTRL0 PHY0*/
/* tocheck	ldr     r1, =0x42190219*/
#ifdef ANDROID_SETTINGS	
	ldreq	r1, =0x42190219		/* solo*/
#else
	ldreq	r1, =0x424C023D		/* solo*/
#endif
	ldrne   r1, =0x43320338		/* quad*/
	ldr     r2, =0x83c
	str     r1, [r0, r2]	
	
	/* setmem /32	0x021b0840 =	0x017B0177	// MPDGCTRL1 PHY0*/
/* tocheck	ldr     r1, =0x017B0177*/
#ifdef ANDROID_SETTINGS	
	ldreq	r1, =0x017B0177		/* solo*/
#else
	ldreq	r1, =0x02330239		/* solo*/
#endif
	ldrne   r1, =0x03360328		/* quad*/
	ldr     r2, =0x840
	str     r1, [r0, r2]

	/* setmem /32	0x021b483c =	0x4201020C	// MPDGCTRL0 PHY1*/
/* tocheck	ldr     r1, =0x4201020C*/
#ifdef ANDROID_SETTINGS
	ldreq	r1, =0x4B4D4E4D		/* solo*/
#else
	ldreq	r1, =0x424C023D		/* solo*/
#endif
	ldrne   r1, =0x43460348		/* quad*/
	ldr     r2, =0x483c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b4840 =	0x01660172	// MPDGCTRL1 PHY1*/
/* tocheck	ldr     r1, =0x01660172*/
#ifdef ANDROID_SETTINGS	
	ldreq	r1, =0x3F3E2D36		/* solo*/
#else
	ldreq	r1, =0x02330239		/* solo*/
#endif
	ldrne   r1, =0x03360308		/* quad*/
	ldr     r2, =0x4840
	str     r1, [r0, r2]
				
/* //Read calibration			*/
	
	/* setmem /32	0x021b0848 =	0x4B4D4E4D	// MPRDDLCTL PHY0*/
/* tocheck	ldr     r1, =0x4B4D4E4D*/
	ldreq	r1, =0x42464745		/* solo*/
	ldrne	r1, =0x3126242B		/* quad*/
	ldr     r2, =0x848
	str     r1, [r0, r2]

	/* setmem /32	0x021b4848 =	0x4A4F5049	// MPRDDLCTL PHY1*/
/* tocheck	ldr     r1, =0x4A4F5049*/
	ldreq   r1, =0x42464745		/* solo*/
	ldrne	r1, =0x2F2F2936		/* quad*/
	ldr     r2, =0x4848
	str     r1, [r0, r2]
				
/* //Write calibration			*/
	
	/* setmem /32	0x021b0850 =	0x3F3E2D36	// MPWRDLCTL PHY0*/
/* tocheck	ldr     r1, =0x3F3E2D36*/
	ldreq   r1, =0x39333138		/* solo*/
	ldrne	r1, =0x3D364240		/* quad*/
	ldr     r2, =0x850
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b4850 =	0x3238372B	// MPWRDLCTL PHY1*/
/* tocheck	ldr     r1, =0x3238372B*/
	ldreq   r1, =0x39333138		/* solo*/
	ldrne	r1, =0x48324943		/* quad*/
	ldr     r2, =0x4850
	str     r1, [r0, r2]		
				
/* //read data bit delay: (3 is the reccommended default value, although out of reset value is 0):			*/
	
	/* setmem /32	0x021b081c =	0x33333333	// DDR_PHY_P0_MPREDQBY0DL3*/
	ldr     r1, =0x33333333
	ldr     r2, =0x81c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b0820 =	0x33333333	// DDR_PHY_P0_MPREDQBY1DL3*/
	ldr     r1, =0x33333333
	ldr     r2, =0x820
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b0824 =	0x33333333	// DDR_PHY_P0_MPREDQBY2DL3*/
	ldr     r1, =0x33333333
	ldr     r2, =0x824
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b0828 =	0x33333333	// DDR_PHY_P0_MPREDQBY3DL3*/
	ldr     r1, =0x33333333
	ldr     r2, =0x828
	str     r1, [r0, r2]

	/* setmem /32	0x021b481c =	0x33333333	// DDR_PHY_P1_MPREDQBY0DL3*/
	ldr     r1, =0x33333333
	ldr     r2, =0x481c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b4820 =	0x33333333	// DDR_PHY_P1_MPREDQBY1DL3*/
	ldr     r1, =0x33333333
	ldr     r2, =0x4820
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b4824 =	0x33333333	// DDR_PHY_P1_MPREDQBY2DL3*/
	ldr     r1, =0x33333333
	ldr     r2, =0x4824
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b4828 =	0x33333333	// DDR_PHY_P1_MPREDQBY3DL3*/
	ldr     r1, =0x33333333
	ldr     r2, =0x4828
	str     r1, [r0, r2]
				
			
/* // Complete calibration by forced measurement:			*/
	
	/* setmem /32	0x021b08b8 =	0x00000800 	// DDR_PHY_P0_MPMUR0, frc_msr*/
	ldr     r1, =0x00000800
	ldr     r2, =0x8b8
	str     r1, [r0, r2]

	/* setmem /32	0x021b48b8 =	0x00000800 	// DDR_PHY_P1_MPMUR0, frc_msr*/
	ldr     r1, =0x00000800
	ldr     r2, =0x48b8
	str     r1, [r0, r2]
				
/* //MMDC init:	*/
#ifdef ANDROID_SETTINGS		
	/* setmem /32	0x021b0004 =	0x00030036	// MMDC0_MDPDC */
	ldr		r1, =0x0002002D
	ldr     r2, =0x0004
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b0008 =	0x09553040	// MMDC0_MDOTC*/
	ldr		r1, =0x09443940
	ldr     r2, =0x0008
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b000c =	0x8A907954	// MMDC0_MDCFG0*/
	ldr		r1, =0x676B5334
	ldr     r2, =0x000c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b0010 =	0xDB738E64	// MMDC0_MDCFG1*/
	ldr     r1, =0xFF538E64
	ldr     r2, =0x0010
	str     r1, [r0, r2]
#else
	/* setmem /32	0x021b0004 =	0x00030036	// MMDC0_MDPDC */
	ldr		r1, =0x00030036
	ldr     r2, =0x0004
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b0008 =	0x09553040	// MMDC0_MDOTC*/
	ldr		r1, =0x09553040
	ldr     r2, =0x0008
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b000c =	0x8A907954	// MMDC0_MDCFG0*/
	ldr		r1, =0x8A907954
	ldr     r2, =0x000c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b0010 =	0xDB738E64	// MMDC0_MDCFG1*/
	ldr     r1, =0xDB738E64
	ldr     r2, =0x0010
	str     r1, [r0, r2]
#endif	
	/* setmem /32	0x021b0014 =	0x01FF00DB	// MMDC0_MDCFG2*/
	ldr     r1, =0x01FF00DB
	ldr     r2, =0x0014
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b0018 =	0x00001740	// MMDC0_MDMISC*/
	ldr     r1, =0x00001740
	ldr     r2, =0x0018
	str     r1, [r0, r2]
 
/* //NOTE about MDMISC RALAT:			*/
/* //MDMISC: RALAT kept to the high level of 5 to ensure stable operation at 528MHz. 			*/
/* //MDMISC: consider reducing RALAT if your 528MHz board design allow that. Lower RALAT benefits: 			*/
/* //a. better operation at low frequency			*/
/* //b. Small performence improvment			*/
			
	/* setmem /32	0x021b001c =	0x00008000	// MMDC0_MDSCR, set the Configuration request bit during MMDC set up*/
	ldr     r1, =0x00008000
	ldr     r2, =0x001c
	str     r1, [r0, r2]
	/* setmem /32	0x021b002c =	0x000026d2	// MMDC0_MDRWD; recommend to maintain the default values*/
	ldr     r1, =0x000026d2
	ldr     r2, =0x002c
	str     r1, [r0, r2]
	/* setmem /32	0x021b0030 =	0x005B0E21	// MMDC0_MDOR*/
	ldr     r1, =0x005B0E21
	ldr     r2, =0x0030
	str     r1, [r0, r2]
	/* setmem /32	0x021b0040 =	0x0000007F	// CS0_END */
	ldr     r1, =0x0000007F
	ldr     r2, =0x0040
	str     r1, [r0, r2]
			
	/* setmem /32	0x021b0000 =	0x84190000	// MMDC0_MDCTL*/
#if defined CONFIG_DDR_16BIT
	ldr     r1, =0x83180000         /* solo 256MByte 16Bit   */
#else	
	ldreq   r1, =0x84190000		/*  solo 1GByte 32Bit    */
    /*  ldreq	r1, =0x83190000		;  solo 512MByte 32Bit*/
#ifndef REDUCE_TO_1GBYTE	
	ldrne	r1, =0x841A0000 	/*  quad 2GByte 64Bit*/
#else	
	ldrne	r1, =0x831A0000		/*  quad 1GByte 64Bit*/
#endif	
#endif
	
	ldr	r2, =0x021BC660		/* Get Fuse 0x660 GP1 */
	ldr	r2, [r2]
	and	r2, r2, #0x30000000	/* GP1 [29..28]       */
	cmp	r2, #0x0		/* Change bus-width? */
	andne	r1, r1, #0xFFFCFFFF	/* Mask bus-width */
	/* cmp	r2, #0x10000000	*/	/* 16bit? */
	/* orreq	r1, r1, */
	cmp	r2, #0x20000000		/* 32bit? */
	orreq	r1, r1, #0x00010000
	cmp	r2, #0x30000000		/* 64bit? */
	orreq	r1, r1, #0x00020000
	
	ldr     r2, =0x0000
	str     r1, [r0, r2]
			
/* // Mode register writes			*/
	
	/* setmem /32	0x021b001c =	0x04088032	// MMDC0_MDSCR, MR2 write, CS0*/
	/*  MR2 = 0x0408*/
					/*                 12    8    4    0*/
	                /*               0000 0100 0000 1000*/
	                /*                     10            Rtt_Wr=RZQ/2*/
	                /*                         0         Self-Refresh: Normal operating temperature*/
	                /*                          0        Manual Self-Refresh*/
	                /*                           00 1    CWL=6*/
	                /*                               000 Full-Array 	           	*/
	ldr     r1, =0x04088032
	ldr     r2, =0x001c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b001c =	0x00008033	// MMDC0_MDSCR, MR3 write, CS0*/
	/*  MR3 = 0  Normal Operation*/
	ldr     r1, =0x00008033
	ldr     r2, =0x001c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b001c =	0x00428031	// MMDC0_MDSCR, MR1 write, CS0*/
	/*  MR1 = 0x0004*/
					/*                 12    8    4    0*/
					/*               0000 0000 0000 0010*/
	                /*                  0                Output Buffer enabled*/
	                /*                    0              TDQS disabled*/
	                /*                      0   0    1   ODT=RZQ/4*/
	                /*                         0         Write-leveling disabled*/
	                /*                            0 0    AL-disabled*/
	                /*                           0    0  Output-Driver=RZQ/7*/
	                /*                                 0 DLL Enabled  	*/
#ifdef ANDROID_SETTINGS	
 	ldr     r1, =0x00428031
#else
 	ldr     r1, =0x00048031
#endif
	ldr     r2, =0x001c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b001c =	0x09308030	// MMDC0_MDSCR, MR0 write, CS0*/
	/*  MR0 = 0x0930*/
					/*                   12    8    4*/
					/* 			     0000 1001 0011 0000*/
					/*                    0				 0: Slow Exit, 1:Fast Exit*/
					/*                      100            WR = 8             */
					/*                         1           DLL-Reset*/
					/*                           0         Normal mode*/
					/*                            011  0   CAS=7*/
					/*                                0    Burst-Type: 0: Nibble Sequentiel, 1:Interleave ??*/
					/*                                  00 Burst=8	*/
	ldr		r1, =0x09308030		/* quad*/
	ldr     r2, =0x001c
	str     r1, [r0, r2]
	
	/* setmem /32	0x021b001c =	0x04008040	// MMDC0_MDSCR, ZQ calibration command sent to device on CS0*/
	ldr     r1, =0x04008040
	ldr     r2, =0x001c
	str     r1, [r0, r2]	
				
							
	/* setmem /32	0x021b0020 =	0x00005800	// MMDC0_MDREF*/
	ldr     r1, =0x00005800
	ldr     r2, =0x0020
	str     r1, [r0, r2]	
	
/* // It is recommended for new board designs and for customer boards			*/
/* // to program these registers to a value of "0x00011117" 			*/
/* // The DRAM ODT remains enabled and it is required to leave the DRAM ODT enabled			*/
			
	/* setmem /32	0x021b0818 =	0x00011117	// DDR_PHY_P0_MPODTCTRL*/
#ifdef ANDROID_SETTINGS
	ldr     r1, =0x00022227
#else
	ldr     r1, =0x00011117
#endif
	ldr     r2, =0x0818
	str     r1, [r0, r2]	

	/* setmem /32	0x021b4818 =	0x00011117	// DDR_PHY_P1_MPODTCTRL*/
	ldr     r1, =0x00011117
	ldr     r2, =0x4818
	str     r1, [r0, r2]	
				
	/* setmem /32	0x021b0004 =	0x00035576	// MMDC0_MDPDC with PWDT bits set*/
	ldr     r1, =0x00035576
	ldr     r2, =0x0004
	str     r1, [r0, r2]	
	
#ifdef DDR_AUTO_POWERDOWN
	/* setmem /32	0x021b0404 = 	0x00011006	// MMDC0_MAPSR ADOPT power down enabled, MMDC will enter automatically to self-refresh while the number of idle cycle reached.*/
	ldr     r1, =0x00011007
	ldr     r2, =0x0404
	str     r1, [r0, r2]
#endif
	
	/* setmem /32	0x021b001c =	0x00000000	// MMDC0_MDSCR, clear this register (especially the configuration bit as initialization is complete)*/
	ldr     r1, =0x00000000
	ldr     r2, =0x001c
	str     r1, [r0, r2]
	/*  end of DDR3 Initialation Routine*/
					
/* ------------------------------------------------------------------------------------------	*/
/* */
/*     The following is to fill in those arguments for this ROM function*/
/*     pu_irom_hwcnfg_setup(void **start, size_t *bytes, const void *boot_data)*/
/* */
/*     This function is used to copy data from the storage media into DDR.*/
/* */
/*     start - Initial (possibly partial) image load address on entry.  */
/*             Final image load address on exit.*/
/*     bytes - Initial (possibly partial) image size on entry.  */
/*             Final image size on exit.*/
/*     boot_data - Initial @ref ivt Boot Data load address.*/
/* */
    /*  Restore the necessary address.*/
PLUGIN_HOOK_ENTRY:	

    MOV R0, R4
    MOV R1, R5
    MOV R2, R6

    /* Get Processor-Skew( Solo/Lite or Dual/Quad) */
    LDR	R4, =0x20C8260	
    LDR	R4,[R4]
    TST	R4, #0x00020000
                       	/* Select version to test against */
    MOVEQ R5, #0X12	/* Solo/Dual-Lite Version 1.2 */
    MOVNE R5, #0x14	/* Dual/Quad Version 1.3: Rev 1.2 = C-marking on chip but 0x13 in ROM-Firmware! */
	                /* Get ROM-Version */
    LDR   R4, =CSP_BASE_MEM_PA_ROM
    LDR   R4, [r4, #ROM_VER_OFFSET]
    /* Set ROM-API-Vector-Offset */
    CMP   R4, R5
    MOV   R5, #0xC4 
    MOVLO R5, #0xC0

    /* ROM API pu_irom_hwcnfg_setup(void **start, size_t *bytes, const void *boot_data)*/
    /* is the 3rd API in the ROM API Vector Table*/
    MOV    R4, R5
    LDR    R4, [R4, #0x08]

    /* We should distinguish USB recovery mode(SDP mode) and internal boot mode.*/
    /* If ROM runs in SDP mode, then it needn't load boot code from storage media.*/
    /* If ROM runs in SDP mode, then r0 must be 0x00*/
    /* If ROM runs in internal boot mode, then r0 should be larger than IRAM base address.    */

    CMP R7, #0x00900000
    BLXHI R4            /*  Address is subject to change in future ROM versions*/

    /* Workaround run plug-ins in SDP mode without USB re-enumeration.*/
    /* how it works:*/
    /* ROM running in usb download mode.*/
    /* Host manufacturing application sends SDP command to download plug-in image.*/
    /* Host manufacturing application sends SDP command to jump to plug-in image and run it.*/
    /* Plug-in starts execution and after its regular tasks plug-in will then call into ROM*/
    /* call into pl_parse_and_handle()*/

    /* ROM API pl_parse_and_handle()*/
    /* is the 2rd API in the ROM API Vector Table*/
    MOV     r4, R5	
    LDR     r4, [R4, #0x4]

    CMP R7, #0x00900000
    BLXLS R4
        
/*  To return to ROM from plugin, we need to fill in these argument.*/
/*  Here is what need to do:*/
/*  Need to construct the paramters for this function before return to ROM:*/
/*  plugin_download(void **start, size_t *bytes, UINT32 *ivt_offset)*/

    /*  Restore registers*/
    /* POPSTACK2LR_ORIG R0, R7 */
    LDMFD   sp!,{r0-r7,lr}

    ldr     r3, DDR_DEST_ADDR
    str     r3, [r0]
    ldr     r3, =BOOT_DATA_IMG_LEN
    str     r3, [r1]
    ldr     r3, =(IVT_OFFSET+(IVT2_ADDR-IVT_ADDR))
    str     r3, [r2]
    mov 	r0, #1

    /* RETURN_THUMB   return back to ROM code*/
    bx lr
	
    /* LTORG */
    /*  Reserved region for CSF data*/
    /*  SECUREDATA */

_FlashHeaderEnd:
    .word 0xdeadbeef

#if 0
    .macro  RESERVE_CSF size
        LCLA    Counter
Counter SETA    \size
        WHILE   Counter > 0
Counter    SETA Counter - 1
           .byte 0x00 
        WEND
    .endm

    .ALIGN 4
CSF_ADDR:	
    .space (IMAGE_HDR_SIZE - (CSF_ADDR - IVT_ADDR)), 0x00
/*  RESERVE_CSF     (IMAGE_HDR_SIZE - (CSF_ADDR - IVT_ADDR)) */

/*  END */
#endif
