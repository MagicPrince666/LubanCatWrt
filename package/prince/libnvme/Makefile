include $(TOPDIR)/rules.mk

PKG_NAME:=libnvme
PKG_VERSION:=master
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=libnvme
PKG_SOURCE_URL:=https://github.com/linux-nvme/libnvme.git
PKG_SOURCE_VERSION:=master
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)
PKG_USE_NINJA:=1

PKG_MAINTAINER:=Keith Busch
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/meson.mk

define Package/$(PKG_NAME)
  SECTION:=libs
  CATEGORY:=Libraries
  TITLE:=libnvme development C library
  URL:=https://github.com/linux-nvme/libnvme.git
  SUBMENU:=Prince
  DEPENDS:= +libjson-c +libuuid
  VARIANT:=
endef

MESON_ARGS += -Db_sanitize=address

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/iw $(1)/usr/bin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
