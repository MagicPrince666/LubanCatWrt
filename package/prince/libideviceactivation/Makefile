#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
# 

include $(TOPDIR)/rules.mk

PKG_NAME:=libideviceactivation
PKG_VERSION:=1.1.1
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=https://github.com/libimobiledevice/libideviceactivation/releases/download/$(PKG_VERSION)
PKG_HASH:=4dfb01068554e889d4fff78f4a7ab4e8d87819148d4e57a9dccbec51b8edc712

PKG_MAINTAINER:=Rosen Penev <rosenp@gmail.com>
PKG_LICENSE:=GPL-2.0-or-later
PKG_LICENSE_FILES:=COPYING

PKG_INSTALL:=1
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/package.mk

define Package/libideviceactivation/Default
	TITLE:=A library that talks to Apple devices.
	URL:=https://www.libimobiledevice.org/
	SUBMENU:=libimobiledevice
endef

define Package/libideviceactivation/Default/description
	A library to manage the activation process of Apple iOS devices.
endef

define Package/libideviceactivation
	$(call Package/libideviceactivation/Default)
	SECTION:=libs
	CATEGORY:=Libraries
	DEPENDS:=+usbmuxd +libplist +libimobiledevice +libxml2 +libcurl
  LICENSE:=LGPL-2.1-or-later
  LICENSE_FILES:=COPYING.LESSER
endef

define Package/libideviceactivation/description
	$(call Package/libideviceactivation/Default/description)
endef

define Package/libideviceactivation-utils
	$(call Package/libideviceactivation/Default)
	SECTION:=utils
	CATEGORY:=Utilities
	DEPENDS:=+libideviceactivation
	LICENSE:=GPL-2.0-or-later
	ICENSE_FILES:=COPYING
endef

define Package/libideviceactivation-utils/description
	$(call Package/libideviceactivation/Default/description)
	This package contains the libideviceactivation utilities.
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_INSTALL_DIR)/usr/include/libideviceactivation.h $(1)/usr/include/
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libideviceactivation-1.0.so* $(1)/usr/lib/
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/libideviceactivation-1.0.pc $(1)/usr/lib/pkgconfig/
endef

define Package/libideviceactivation/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libideviceactivation-1.0.so* $(1)/usr/lib/
endef

define Package/libideviceactivation-utils/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(CP) $(PKG_INSTALL_DIR)/usr/bin/ideviceactivation $(1)/usr/bin/
endef

$(eval $(call BuildPackage,libideviceactivation))
$(eval $(call BuildPackage,libideviceactivation-utils))
