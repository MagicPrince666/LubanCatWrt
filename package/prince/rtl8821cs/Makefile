include $(TOPDIR)/rules.mk

PKG_NAME:=rtl8821cs-sdio
PKG_VERSION:=main
PKG_RELEASE=1

PKG_LICENSE:=GPLv2
PKG_LICENSE_FILES:=COPYING

PKG_SOURCE_URL:=https://github.com/MagicPrince666/rtl8821cs.git
PKG_MIRROR_HASH:=4e44086c2e80c45ed6c2e3338bda56ad4313acc650c95381faf222c1f8340961
PKG_SOURCE_PROTO:=git
PKG_SOURCE_DATE:=2023-08-23
PKG_SOURCE_VERSION:=main

PKG_MAINTAINER:=leo
PKG_BUILD_PARALLEL:=1

STAMP_CONFIGURED_DEPENDS := $(STAGING_DIR)/usr/include/mac80211-backport/backport/autoconf.h

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/$(PKG_NAME)
	SUBMENU:=Wireless Drivers
	TITLE:=Realtek 8821CS SDIO or SPI WiFi
	DEPENDS:=+kmod-cfg80211 +@DRIVER_11N_SUPPORT
	KCONFIG:= \
		CONFIG_RTL8821CS=m
	FILES:=$(PKG_BUILD_DIR)/8821cs.ko
	AUTOLOAD:=$(call AutoProbe,8821cs)
endef

NOSTDINC_FLAGS := \
	$(KERNEL_NOSTDINC_FLAGS) \
	-I$(PKG_BUILD_DIR) \
	-I$(PKG_BUILD_DIR)/include \
	-I$(PKG_BUILD_DIR)/hal/phydm \
	-I$(PKG_BUILD_DIR)/platform \
	-I$(PKG_BUILD_DIR)/hal/btc \
	-I$(STAGING_DIR)/usr/include \
	-I$(STAGING_DIR)/usr/include/mac80211-backport \
	-I$(STAGING_DIR)/usr/include/mac80211-backport/uapi \
	-I$(STAGING_DIR)/usr/include/mac80211 \
	-I$(STAGING_DIR)/usr/include/mac80211/uapi \
	-include backport/backport.h \
	-Wno-error=address \
	-Wno-error=stringop-overread

NOSTDINC_FLAGS+=-DCONFIG_IOCTL_CFG80211 -DRTW_USE_CFG80211_STA_EVENT -DBUILD_OPENWRT

define Build/Compile
	$(MAKE) ARCH=$(LINUX_KARCH) CROSS_COMPILE=$(TARGET_CROSS) -C $(LINUX_DIR) M=$(PKG_BUILD_DIR) EXTRA_CFLAGS="$(NOSTDINC_FLAGS)" CONFIG_RTL8821CS=m modules
endef

$(eval $(call KernelPackage,$(PKG_NAME)))
