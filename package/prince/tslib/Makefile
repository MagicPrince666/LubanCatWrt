#
# Copyright (C) 2012-2014 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
# https://github.com/libts/tslib/releases/download/1.22/tslib-1.22.tar.bz2

include $(TOPDIR)/rules.mk

PKG_NAME:=tslib
PKG_VERSION:=1.22
PKG_RELEASE:=2

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=https://github.com/libts/tslib/releases/download/$(PKG_VERSION)/
PKG_HASH:=77aa04e6658d711f1d8141ada01878d34e10bdcaff590341cdea354b6a1b15cc

PKG_MAINTAINER:=libts
PKG_LICENSE:=MIT
PKG_LICENSE_FILES:=COPYING

PKG_INSTALL:=1
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

define Package/$(PKG_NAME)
  SECTION:=libs
  CATEGORY:=Libraries
  DEPENDS:=
  TITLE:=C library for filtering touchscreen events
  URL:=https://github.com/libts/tslib
  SUBMENU:=Prince
endef

CONFIGURE_ARGS += $(FPIC)

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig $(1)/usr/include/
	$(CP) $(PKG_INSTALL_DIR)/usr/include/* $(1)/usr/include/
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/libts.so* $(1)/usr/lib/
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/tslib.pc $(1)/usr/lib/pkgconfig
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_BUILD_DIR)/src/libts.so* $(1)/usr/lib/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
