#!/bin/sh
../tools/mkimage -A arm -T script -O linux -d  boot.debian_3.0.35.txt                 boot.debian_3.0.35.scr
../tools/mkimage -A arm -T script -O linux -d  boot.kernel-dhcp-tftp.txt              boot.kernel-dhcp-tftp.scr
../tools/mkimage -A arm -T script -O linux -d  boot.kernel-dhcp-tftp-debian-ipan7.txt boot.kernel-dhcp-tftp-debian-ipan7.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian-ipan10-10inch.txt          boot.debian-ipan10-10inch.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian-ica-xga.txt                boot.debian-ica-xga.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian-ica-ipan5.txt              boot.debian-ica-ipan5.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian-ipan10-7inch.txt           boot.debian-ipan10-7inch.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.ipan7.txt                  boot.debian.ipan7.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.ipan7-mmcblk3.txt          boot.debian.ipan7-mmcblk3.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.pConXS-mmc3.txt            boot.debian.pConXS-mmc3.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.pConXS.txt                 boot.debian.pConXS.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.ipant7.txt                 boot.debian.ipant7.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.nfs.txt                    boot.debian.nfs.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.txt                        boot.debian.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.tty.txt                    boot.debian.tty.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.powersave.txt              boot.debian.powersave.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.hdmi.txt                   boot.debian.hdmi.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.hdmi-fullhd.txt            boot.debian.hdmi-fullhd.scr
../tools/mkimage -A arm -T script -O linux -d  boot.usb.txt                           boot.usb.scr
../tools/mkimage -A arm -T script -O linux -d  boot.android.txt                       boot.android.scr
../tools/mkimage -A arm -T script -O linux -d  boot.android-nologo.txt                boot.android-nologo.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.5.4.txt                    boot.debian.5.4.scr