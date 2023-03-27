#
# Copyright (C) 2012-2014 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
# https://github.com/knik0/faac/archive/refs/tags/1_30.tar.gz

include $(TOPDIR)/rules.mk

PKG_NAME:=faac
PKG_VERSION:=1_30
PKG_RELEASE:=

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=https://github.com/knik0/faac/archive/refs/tags/1_30
PKG_HASH:=adc387ce588cca16d98c03b6ec1e58f0ffd9fc6eadb00e254157d6b16203b2d2

PKG_MAINTAINER:=libfaac
PKG_LICENSE:=MIT
PKG_LICENSE_FILES:=COPYING

PKG_FIXUP:=libtool
PKG_INSTALL:=1
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
  SECTION:=libs
  CATEGORY:=Libraries
  DEPENDS:=
  TITLE:=C library for pcm encode aac
  URL:=https://github.com/knik0/faac
  SUBMENU:=Prince
endef

define Build/Configure
	cd $(PKG_BUILD_DIR); \
		./bootstrap ; \
		autoreconf -fiv
	$(call Build/Configure/Default)
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/include/* $(1)/usr/include/
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/libfaac.so* $(1)/usr/lib/
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libfaac.so* $(1)/usr/lib/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
