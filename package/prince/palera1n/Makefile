include $(TOPDIR)/rules.mk

PKG_NAME:=palera1n
PKG_VERSION:=main
PKG_RELEASE:=$(AUTORELEASE)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_NAME:=palera1n
PKG_SOURCE_URL:=https://github.com/MagicPrince666/palera1n.git
PKG_MIRROR_HASH:=5a67e31808a571c77a176386bc340ebd1472a74cd60d2dfa76dc608290edf62e
PKG_SOURCE_VERSION:=$(PKG_VERSION)
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_SOURCE_NAME)-$(PKG_VERSION)

PKG_INSTALL:=1

PKG_MAINTAINER:=palera1n team

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:=iOS 15+ arm64 iOS/iPadOS jailbreaking tool
	SUBMENU:=Prince
	DEPENDS:=+libpthread +libimobiledevice +libplist +libreadline +libirecovery \
		+libmbedtls +libnewt +libusb-1.0 +libusbmuxd +libslang2 +libpopt +librt
endef

define Package/$(PKG_NAME)/description
	iOS 15+ arm64 iOS/iPadOS jailbreaking tool
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(CP) $(PKG_BUILD_DIR)/src/palera1n $(1)/usr/bin/
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
