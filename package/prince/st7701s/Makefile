#
# Copyright (C) 2006-2009 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=fb_st7701s

PKG_REV:=huangliquan
PKG_VERSION:=$(PKG_REV)
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)

include $(INCLUDE_DIR)/package.mk

define KernelPackage/$(PKG_NAME)
  SUBMENU:=Prince Modules
  TITLE:=Sitronix ST7701S fb driver
  FILES:=$(PKG_BUILD_DIR)/fb_st7701s.ko
  AUTOLOAD:=$(call AutoLoad,50,fb_st7701s)
endef

define KernelPackage/$(PKG_NAME)/description
 Sitronix ST7701S fb driver
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

