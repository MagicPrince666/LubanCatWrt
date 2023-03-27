#
# Copyright (C) 2020 - 2021 HandsomeMod Project
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=esp32-sdio
PKG_RELEASE:=2

STAMP_CONFIGURED_DEPENDS := $(STAGING_DIR)/usr/include/mac80211-backport/backport/autoconf.h

include $(INCLUDE_DIR)/package.mk

define KernelPackage/$(PKG_NAME)
  SUBMENU:=Wireless Drivers
  TITLE:=Esp32 cfg80211 wifi driver
  DEPENDS:=+kmod-cfg80211 +kmod-mac80211 +kmod-bluetooth +@DRIVER_11N_SUPPORT +@DRIVER_11AC_SUPPORT
  MAINTAINER:=Huangliquan <846863428@qq.com>
  KCONFIG:= CONFIG_ESP32_SDIO=m
  FILES:=$(PKG_BUILD_DIR)/esp32_sdio.ko
  AUTOLOAD:=$(call AutoProbe,esp32_sdio)
endef

include $(INCLUDE_DIR)/kernel-defaults.mk

NOSTDINC_FLAGS = \
	-I$(PKG_BUILD_DIR) \
	-I$(PKG_BUILD_DIR)/include \
	-I$(STAGING_DIR)/usr/include/mac80211-backport \
	-I$(STAGING_DIR)/usr/include/mac80211-backport/uapi \
	-I$(STAGING_DIR)/usr/include/mac80211 \
	-I$(STAGING_DIR)/usr/include/mac80211/uapi \
	-include backport/backport.h

#NOSTDINC_FLAGS+=-DESP_ACK_INTERRUPT

define Build/Prepare
	$(call Build/Prepare/Default)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	+$(MAKE) $(PKG_JOBS) -C "$(LINUX_DIR)" \
		$(KERNEL_MAKE_FLAGS) \
		M="$(PKG_BUILD_DIR)" \
		NOSTDINC_FLAGS="$(NOSTDINC_FLAGS)" \
		modules
endef


$(eval $(call KernelPackage,$(PKG_NAME)))
