#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
# 

include $(TOPDIR)/rules.mk

PKG_NAME:=ideviceinstaller
PKG_VERSION:=1.1.1
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=https://github.com/libimobiledevice/ideviceinstaller/releases/download/$(PKG_VERSION)
PKG_HASH:=deb883ec97f2f88115aab39f701b83c843e9f2b67fe02f5e00a9a7d6196c3063

PKG_MAINTAINER:=Rosen Penev <rosenp@gmail.com>
PKG_LICENSE:=GPL-2.0-or-later
PKG_LICENSE_FILES:=COPYING

PKG_INSTALL:=1
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/package.mk

define Package/ideviceinstaller
  SECTION:=utils
  CATEGORY:=Utilities
  SUBMENU:=libimobiledevice
  TITLE:=manage apps and app archives
  URL:=https://github.com/libimobiledevice/ideviceinstaller
  DEPENDS:=+usbmuxd +libplist +libimobiledevice +libzip
endef

define Package/ideviceinstaller/description
  A command-line application to manage apps and app archives on iOS devices.
endef

define Package/ideviceinstaller/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(CP) $(PKG_INSTALL_DIR)/usr/bin/ideviceinstaller $(1)/usr/bin/
endef

$(eval $(call BuildPackage,ideviceinstaller))
