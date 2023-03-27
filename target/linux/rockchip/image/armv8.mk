# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2020 Tobias Maedel

# FIT will be loaded at 0x02080000. Leave 16M for that, align it to 2M and load the kernel after it.
KERNEL_LOADADDR := 0x03200000

define Device/embedfire_doornet1
  DEVICE_VENDOR := EmbedFire
  DEVICE_MODEL := DoorNet1
  SOC := rk3328
  UBOOT_DEVICE_NAME := doornet1-rk3328
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk3328 | pine64-bin | gzip | append-metadata
  DEVICE_PACKAGES := kmod-usb-net-rtl8152 kmod-rtl8821cu
endef
TARGET_DEVICES += embedfire_doornet1

define Device/embedfire_doornet2
  DEVICE_VENDOR := EmbedFire
  DEVICE_MODEL := DoorNet2
  SOC := rk3399
  UBOOT_DEVICE_NAME := doornet2-rk3399
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk3399 | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := kmod-r8169 -urngd
endef
TARGET_DEVICES += embedfire_doornet2

define Device/embedfire_lubancat-zero-n
  DEVICE_VENDOR := EmbedFire
  DEVICE_MODEL := LubanCat Zero N
  SOC := rk3566
  UBOOT_DEVICE_NAME := lubancat-zero-n-rk3566
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk356x | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := kmod-r8125
endef
TARGET_DEVICES += embedfire_lubancat-zero-n

define Device/embedfire_lubancat-zero-w
  DEVICE_VENDOR := EmbedFire
  DEVICE_MODEL := LubanCat Zero W
  SOC := rk3566
  UBOOT_DEVICE_NAME := lubancat-zero-w-rk3566
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk356x | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := kmod-r8125
endef
TARGET_DEVICES += embedfire_lubancat-zero-w

define Device/embedfire_lubancat1
  DEVICE_VENDOR := EmbedFire
  DEVICE_MODEL := LubanCat 1
  SOC := rk3566
  UBOOT_DEVICE_NAME := lubancat1-rk3566
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk356x | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := kmod-r8169
endef
TARGET_DEVICES += embedfire_lubancat1

define Device/embedfire_lubancat1n
  DEVICE_VENDOR := EmbedFire
  DEVICE_MODEL := LubanCat 1N
  SOC := rk3566
  UBOOT_DEVICE_NAME := lubancat1n-rk3566
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk356x | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := kmod-r8169 -urngd kmod-ata-ahci
endef
TARGET_DEVICES += embedfire_lubancat1n

define Device/embedfire_lubancat2
  DEVICE_VENDOR := EmbedFire
  DEVICE_MODEL := LubanCat 2
  SOC := rk3568
  UBOOT_DEVICE_NAME := lubancat2-rk3568
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk356x | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := kmod-ata-ahci kmod-ata-ahci-platform kmod-ata-core
endef
TARGET_DEVICES += embedfire_lubancat2

define Device/embedfire_lubancat2n
  DEVICE_VENDOR := EmbedFire
  DEVICE_MODEL := LubanCat 2N
  SOC := rk3568
  UBOOT_DEVICE_NAME := lubancat2n-rk3568
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk356x | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := kmod-r8125 kmod-ata-ahci kmod-ata-ahci-platform kmod-ata-core
endef
TARGET_DEVICES += embedfire_lubancat2n

define Device/friendlyarm_nanopi-r2s
  DEVICE_VENDOR := FriendlyARM
  DEVICE_MODEL := NanoPi R2S
  SOC := rk3328
  UBOOT_DEVICE_NAME := nanopi-r2s-rk3328
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk3328 | pine64-bin | gzip | append-metadata
  DEVICE_PACKAGES := kmod-usb-net-rtl8152
endef
TARGET_DEVICES += friendlyarm_nanopi-r2s

define Device/friendlyarm_nanopi-r4s
  DEVICE_VENDOR := FriendlyARM
  DEVICE_MODEL := NanoPi R4S
  SOC := rk3399
  UBOOT_DEVICE_NAME := nanopi-r4s-rk3399
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk3399 | pine64-bin | gzip | append-metadata
  DEVICE_PACKAGES := kmod-r8169 -urngd
endef
TARGET_DEVICES += friendlyarm_nanopi-r4s

define Device/pine64_rockpro64
  DEVICE_VENDOR := Pine64
  DEVICE_MODEL := RockPro64
  SOC := rk3399
  UBOOT_DEVICE_NAME := rockpro64-rk3399
  IMAGE/sysupgrade.img.gz := boot-common | boot-script | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := -urngd
endef
TARGET_DEVICES += pine64_rockpro64

define Device/radxa_rock-3a
  DEVICE_VENDOR := Radxa
  DEVICE_MODEL := ROCK3 Model A
  SOC := rk3568
  SUPPORTED_DEVICES := radxa,rock3a
  UBOOT_DEVICE_NAME := rock-3a-rk3568
  IMAGE/sysupgrade.img.gz := boot-common | boot-script rk356x | pine64-img | gzip | append-metadata
endef
TARGET_DEVICES += radxa_rock-3a

define Device/radxa_rock-pi-4
  DEVICE_VENDOR := Radxa
  DEVICE_MODEL := ROCK Pi 4
  SOC := rk3399
  SUPPORTED_DEVICES := radxa,rockpi4
  UBOOT_DEVICE_NAME := rock-pi-4-rk3399
  IMAGE/sysupgrade.img.gz := boot-common | boot-script | pine64-img | gzip | append-metadata
  DEVICE_PACKAGES := -urngd
endef
TARGET_DEVICES += radxa_rock-pi-4