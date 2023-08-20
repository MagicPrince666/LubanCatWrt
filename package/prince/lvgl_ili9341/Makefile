include $(TOPDIR)/rules.mk

PKG_NAME:=lvgl_ili9341
PKG_VERSION:=master
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=lvgl_ili9341
PKG_SOURCE_URL:=https://github.com/lvgl/lv_port_linux_frame_buffer
PKG_SOURCE_VERSION:=master
PKG_MIRROR_HASH:=c9a2329f4533e298a312f3271811ee5a895a79e56655e4e6e9383b6092be6d25
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)
PKG_USE_NINJA:=1

PKG_MAINTAINER:=Littlev Graphics Library
PKG_LICENSE:=MIT

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
  SECTION:=utils
	CATEGORY:=Utilities
  TITLE:=LVGL for frame buffer device
  URL:=https://github.com/lvgl/lv_port_linux_frame_buffer.git
  SUBMENU:=Prince
  DEPENDS:=
  VARIANT:=
endef

TARGET_CFLAGS += -I$(PKG_BUILD_DIR)

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/demo $(1)/usr/bin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
