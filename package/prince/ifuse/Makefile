#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=ifuse
PKG_VERSION:=1.1.4
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=https://github.com/libimobiledevice/ifuse/releases/download/$(PKG_VERSION)
PKG_HASH:=3550702ef94b2f5f16c7db91c6b3282b2aed1340665834a03e47458e09d98d87

PKG_MAINTAINER:=Rosen Penev <rosenp@gmail.com>
PKG_LICENSE:=GPL-2.0-or-later
PKG_LICENSE_FILES:=COPYING

PKG_INSTALL:=1
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/package.mk

define Package/ifuse
  SECTION:=utils
  CATEGORY:=Utilities
  SUBMENU:=libimobiledevice
  TITLE:=Mount media directory of an iOS device locally
  URL:=https://github.com/libimobiledevice/ifuse
  DEPENDS:=+libirecovery +usbmuxd +libplist +libimobiledevice +libfuse
endef

define Package/ifuse/description
  A fuse filesystem implementation to access the contents of iOS devices.
endef

TARGET_LDFLAGS += $(if $(CONFIG_USE_GLIBC),-lpthread)

define Package/ifuse/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(CP) $(PKG_INSTALL_DIR)/usr/bin/ifuse $(1)/usr/bin/
endef

$(eval $(call BuildPackage,ifuse))
