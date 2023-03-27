#
# Copyright (C) 2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=spi-can-mcp25xxfd
PKG_RELEASE:=spi-can-mcp25xxfd

include $(INCLUDE_DIR)/package.mk

define KernelPackage/$(PKG_NAME)
  SUBMENU:=Prince Modules
  TITLE:=Microchip spi can mcp25xxfd driver
  FILES:=$(PKG_BUILD_DIR)/mcp25xxfd.ko
  KCONFIG:=
  AUTOLOAD:=$(call AutoProbe,mcp25xxfd)
  DEPENDS:=
endef

define KernelPackage/$(PKG_NAME)/description
 Microchip spi can mcp25xxfd driver
endef

EXTRA_KCONFIG:= \
	CONFIG_CAN_MCP25XXFD=m

EXTRA_CFLAGS:= \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=m,%,$(filter %=m,$(EXTRA_KCONFIG)))) \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=y,%,$(filter %=y,$(EXTRA_KCONFIG)))) \

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)" \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(EXTRA_KCONFIG)

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C $(MAKE_OPTS) modules
endef

$(eval $(call KernelPackage,$(PKG_NAME)))
