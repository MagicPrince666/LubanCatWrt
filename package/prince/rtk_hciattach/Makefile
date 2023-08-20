include $(TOPDIR)/rules.mk

PKG_NAME:=rtk_hciattach
PKG_VERSION:=1.0.0
PKG_RELEASE:=1
PKG_BUILD_DIR:= $(BUILD_DIR)/$(PKG_NAME)


include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)/Default
	SECTION:=utils
	CATEGORY:=Utilities
	SUBMENU:=Prince
endef

define Package/$(PKG_NAME)/description/Default
	HCI UART initialization tool for realtek BT module
endef

define Package/rtk_hciattach
	$(Package/$(PKG_NAME)/Default)
	TITLE:=Realtek BT HCI UART initialization tools
	DEPENDS:=+bluez-utils +libc
endef


define Package/rtk_hciattach/description
	$(call Package/$(PKG_NAME)/description/Default)
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) -r ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Configure
endef

define Package/rtk_hciattach/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/rtk_hciattach $(1)/usr/bin/
endef

$(eval $(call BuildPackage,rtk_hciattach))
