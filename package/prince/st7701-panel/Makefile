#
# Copyright (C) 2006-2009 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=panel-sitronix-st7701

PKG_REV:=huangliquan
PKG_VERSION:=$(PKG_REV)
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)

include $(INCLUDE_DIR)/package.mk

define KernelPackage/$(PKG_NAME)
  SUBMENU:=Prince Modules
  TITLE:=Sitronix ST7701 panel driver
  DEPENDS:=+kmod-fb-tft +kmod-drm +kmod-drm-kms-helper +kmod-drm-dma-helper +kmod-drm-mipi-dbi
  FILES:=$(PKG_BUILD_DIR)/panel-sitronix-st7701.ko
  AUTOLOAD:=$(call AutoLoad,50,panel-sitronix-st7701)
endef

define KernelPackage/$(PKG_NAME)/description
 Sitronix ST7701 panel driver
endef

define KernelPackage/$(PKG_NAME)/config
#	source "$(SOURCE)/config.in"
endef

SOURCE_DIR:=$(PKG_BUILD_DIR)
export SOURCE_DIR

EXTRA_CFLAGS :=

MAKEOPTS:= -C $(LINUX_DIR) \
                ARCH="$(LINUX_KARCH)" \
                CROSS_COMPILE="$(TARGET_CROSS)" \
                M="$(PKG_BUILD_DIR)" \
                EXTRA_CFLAGS="$(EXTRA_CFLAGS)"

define Build/Prepare
	$(call Build/Prepare/Default)
	$(CP) -r src/* $(PKG_BUILD_DIR)
endef

define Build/Compile
	$(MAKE) $(MAKEOPTS)
endef

$(eval $(call KernelPackage,$(PKG_NAME)))

