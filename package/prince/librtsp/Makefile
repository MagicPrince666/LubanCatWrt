include $(TOPDIR)/rules.mk

PKG_NAME:=librtsp
PKG_VERSION:=master
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=librtsp
PKG_SOURCE_URL:=https://github.com/MagicPrince666/librtsp.git
PKG_SOURCE_VERSION:=master
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)

PKG_INSTALL:=1

PKG_MAINTAINER:=Leo huang

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:=librtsp rtsp video transfer
	SUBMENU:=Prince
	DEPENDS:=+libstdcpp +libpthread +libx264
endef

define Package/$(PKG_NAME)/description
	rtsp video transfer
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(CP) $(PKG_BUILD_DIR)/rtsp_demo $(1)/usr/bin/
	$(CP) $(PKG_BUILD_DIR)/V4l2Demo $(1)/usr/bin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
