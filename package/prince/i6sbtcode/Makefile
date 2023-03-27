include $(TOPDIR)/rules.mk

PKG_NAME:=rkble
PKG_VERSION:=main
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=rkble
PKG_SOURCE_URL:=https://github.com/MagicPrince666/rkble.git
PKG_MIRROR_HASH:=585b9f1b7ea78e5c6081109d417d8148fa69a9adc6912d3ac85fe2203310ddbd
PKG_SOURCE_VERSION:=main
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)

PKG_INSTALL:=1

PKG_MAINTAINER:=Leo huang

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:=cputools i6s bluetooth tools
	SUBMENU:=Prince
	DEPENDS:=+libstdcpp +libimobiledevice +libplist +bluez-libs +libpthread +librt +glib2 +libncurses +libreadline $(INTL_DEPENDS) $(ICONV_DEPENDS) +dbus
endef

define Package/$(PKG_NAME)/description
	cputools i6s bluetooth tools
endef

TARGET_CFLAGS += -I$(STAGING_DIR)/usr/include/dbus-1.0 -I$(STAGING_DIR)/usr/lib/dbus-1.0/include -I$(STAGING_DIR)/usr/include/glib-2.0
TARGET_LDFLAGS += -L$(STAGING_DIR)/usr/lib

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(CP) $(PKG_BUILD_DIR)/bt_cputools $(1)/usr/sbin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
