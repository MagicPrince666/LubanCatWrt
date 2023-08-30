include $(TOPDIR)/rules.mk

PKG_NAME:=minilidar
PKG_VERSION:=main
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=minilidar
PKG_SOURCE_URL:=https://github.com/MagicPrince666/minilidar.git
PKG_SOURCE_VERSION:=main
PKG_MIRROR_HASH:=37d11f9d03501736b9dec363a7adb29baffed8dd8109a2561496c696cd26c95d
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)

PKG_INSTALL:=1

PKG_MAINTAINER:=Leo huang

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:=minilidar visualization lidar
	SUBMENU:=Prince
	DEPENDS:=+libstdcpp +libpthread
endef

define Package/$(PKG_NAME)/description
	minilidar visualization lidar
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(CP) $(PKG_BUILD_DIR)/minilidar $(1)/usr/bin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
