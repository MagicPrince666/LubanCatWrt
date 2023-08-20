include $(TOPDIR)/rules.mk

PKG_NAME:=rtl8723ds-sdio
PKG_VERSION:=master
PKG_RELEASE=1

PKG_LICENSE:=GPLv2
PKG_LICENSE_FILES:=COPYING

PKG_SOURCE_URL:=https://github.com/lwfinger/rtl8723ds.git
PKG_MIRROR_HASH:=1b49c2a52008c6bf4ade02c71924cedc946874d55ea4bf3295800a6682a77008
PKG_SOURCE_PROTO:=git
PKG_SOURCE_DATE:=2023-04-07
PKG_SOURCE_VERSION:=master

PKG_MAINTAINER:=lwfinger
PKG_BUILD_PARALLEL:=1

STAMP_CONFIGURED_DEPENDS := $(STAGING_DIR)/usr/include/mac80211-backport/backport/autoconf.h

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/$(PKG_NAME)
	SUBMENU:=Wireless Drivers
	TITLE:=Realtek 8723DS SDIO or SPI WiFi
	DEPENDS:=+kmod-cfg80211 +rtl8723ds-firmware +@DRIVER_11N_SUPPORT
	KCONFIG:= \
		CONFIG_RTL8723DS=m
	FILES:=$(PKG_BUILD_DIR)/8723ds.ko
	AUTOLOAD:=$(call AutoProbe,8723ds)
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

NOSTDINC_FLAGS+=-DCONFIG_IOCTL_CFG80211 -DCONFIG_LITTLE_ENDIAN -DCONFIG_CONCURRENT_MODE -DCONFIG_PLATFORM_HISILICON -DRTW_USE_CFG80211_STA_EVENT -DBUILD_OPENWRT

define Build/Compile
	$(MAKE) ARCH=$(LINUX_KARCH) CROSS_COMPILE=$(TARGET_CROSS) -C $(LINUX_DIR) M=$(PKG_BUILD_DIR) EXTRA_CFLAGS="$(NOSTDINC_FLAGS) -DDM_ODM_SUPPORT_TYPE=0x04" CONFIG_RTL8723DS=m modules
endef

$(eval $(call KernelPackage,$(PKG_NAME)))
