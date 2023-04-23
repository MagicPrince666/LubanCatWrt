include $(TOPDIR)/rules.mk

PKG_NAME:=canfd
PKG_VERSION:=master
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=$(PKG_NAME)
PKG_SOURCE_URL:=https://github.com/MagicPrince666/canfd.git
PKG_SOURCE_VERSION:=$(PKG_VERSION)
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)
PKG_USE_NINJA:=1

PKG_MAINTAINER:=leo huang
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
  SECTION:=libs
  CATEGORY:=Utilities
  TITLE:=canfd c++ demo
  URL:=https://github.com/MagicPrince666/canfd.git
  SUBMENU:=Prince
  DEPENDS:= +libpthread +libstdcpp
  VARIANT:=
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/canfd $(1)/usr/bin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
