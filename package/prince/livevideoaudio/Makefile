include $(TOPDIR)/rules.mk

PKG_NAME:=livevideoaudio
PKG_VERSION:=main
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=livevideoaudio
PKG_SOURCE_URL:=https://github.com/MagicPrince666/rtsp_h264_aac.git
PKG_MIRROR_HASH:=e1e334f11863b382f0254239c88a48756a43242db0d8c950ddfec1f845c93db9
PKG_SOURCE_VERSION:=main
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)

PKG_INSTALL:=1

PKG_MAINTAINER:=Leo huang

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:=a rtsp demo with video and audio
	SUBMENU:=Prince
	DEPENDS:=+libstdcpp +libpthread +alsa-lib +libx264 +faac
endef

define Package/$(PKG_NAME)/description
	a rtsp demo with video and audio
endef

CMAKE_OPTIONS += 

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/sbin $(1)/etc/init.d
	$(CP) $(PKG_BUILD_DIR)/LiveVideo $(1)/usr/sbin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
