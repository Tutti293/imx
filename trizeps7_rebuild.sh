#!/bin/bash
source /opt/fsl-imx-fb/5.4-zeus/environment-setup-cortexa9t2hf-neon-poky-linux-gnueabi
make distclean
make trizeps7_defconfig
make -j$(nproc)
