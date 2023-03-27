include $(TOPDIR)/rules.mk

PKG_NAME:=lvgl_ili9341
PKG_VERSION:=master
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=lvgl_ili9341
PKG_SOURCE_URL:=https://github.com/lvgl/lv_port_linux_frame_buffer
PKG_SOURCE_VERSION:=master
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)
PKG_USE_NINJA:=1

PKG_MAINTAINER:=Littlev Graphics Library
PKG_LICENSE:=MIT

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
  SECTION:=libs
  CATEGORY:=Libraries
  TITLE:=LVGL for frame buffer device
  URL:=https://github.com/lvgl/lv_port_linux_frame_buffer.git
  SUBMENU:=Prince
  DEPENDS:= +tslib
  VARIANT:=
endef

TARGET_CFLAGS += -I$(PKG_BUILD_DIR)

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/demo $(1)/usr/sbin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
