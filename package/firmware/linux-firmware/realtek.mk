Package/r8152-firmware = $(call Package/firmware-default,RealTek RTL8152 firmware)
define Package/r8152-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtl_nic
	$(CP) \
		$(PKG_BUILD_DIR)/rtl_nic/rtl8153* \
		$(PKG_BUILD_DIR)/rtl_nic/rtl8156* \
		$(1)/lib/firmware/rtl_nic
endef
$(eval $(call BuildPackage,r8152-firmware))

Package/r8169-firmware = $(call Package/firmware-default,RealTek RTL8169 firmware)
define Package/r8169-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtl_nic
	$(CP) \
		$(PKG_BUILD_DIR)/rtl_nic/rtl810* \
		$(PKG_BUILD_DIR)/rtl_nic/rtl8125* \
		$(PKG_BUILD_DIR)/rtl_nic/rtl8168* \
		$(PKG_BUILD_DIR)/rtl_nic/rtl84* \
		$(1)/lib/firmware/rtl_nic
endef
$(eval $(call BuildPackage,r8169-firmware))

Package/rtl8188eu-firmware = $(call Package/firmware-default,RealTek RTL8188EU firmware)
define Package/rtl8188eu-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(CP) \
		$(PKG_BUILD_DIR)/rtlwifi/rtl8188eufw.bin \
		$(1)/lib/firmware/rtlwifi
endef
$(eval $(call BuildPackage,rtl8188eu-firmware))

Package/rtl8192ce-firmware = $(call Package/firmware-default,RealTek RTL8192CE firmware)
define Package/rtl8192ce-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192cfw.bin $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192cfwU.bin $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192cfwU_B.bin $(1)/lib/firmware/rtlwifi
endef
$(eval $(call BuildPackage,rtl8192ce-firmware))

Package/rtl8192cu-firmware = $(call Package/firmware-default,RealTek RTL8192CU firmware)
define Package/rtl8192cu-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192cufw.bin $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192cufw_A.bin $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192cufw_B.bin $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192cufw_TMSC.bin $(1)/lib/firmware/rtlwifi
endef
$(eval $(call BuildPackage,rtl8192cu-firmware))

Package/rtl8192de-firmware = $(call Package/firmware-default,RealTek RTL8192DE firmware)
define Package/rtl8192de-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192defw.bin $(1)/lib/firmware/rtlwifi
endef
$(eval $(call BuildPackage,rtl8192de-firmware))

Package/rtl8192eu-firmware = $(call Package/firmware-default,RealTek RTL8192EU firmware)
define Package/rtl8192eu-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192eu_nic.bin $(1)/lib/firmware/rtlwifi
endef
$(eval $(call BuildPackage,rtl8192eu-firmware))

Package/rtl8192se-firmware = $(call Package/firmware-default,RealTek RTL8192SE firmware)
define Package/rtl8192se-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8192sefw.bin $(1)/lib/firmware/rtlwifi
endef
$(eval $(call BuildPackage,rtl8192se-firmware))

Package/rtl8723au-firmware = $(call Package/firmware-default,RealTek RTL8723AU firmware)
define Package/rtl8723au-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8723aufw_A.bin $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8723aufw_B.bin $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8723aufw_B_NoBT.bin $(1)/lib/firmware/rtlwifi
endef
$(eval $(call BuildPackage,rtl8723au-firmware))

Package/rtl8723bu-firmware = $(call Package/firmware-default,RealTek RTL8723BU firmware)
define Package/rtl8723bu-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8723bu_nic.bin $(1)/lib/firmware/rtlwifi
	ln -s rtl8723bu_nic.bin $(1)/lib/firmware/rtlwifi/rtl8723bs_nic.bin
endef
$(eval $(call BuildPackage,rtl8723bu-firmware))

Package/rtl8723ds_bt-firmware = $(call Package/firmware-default,RealTek RTL8723DS_BT Bluetooth firmware)
define Package/rtl8723ds_bt-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtl_bt
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtl_bt/rtl8723d_fw.bin $(1)/lib/firmware/rtl_bt
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtl_bt/rtl8723d_config.bin $(1)/lib/firmware/rtl_bt
	ln -s rtl8723d_fw.bin $(1)/lib/firmware/rtl_bt/rtl8723ds_fw.bin
	ln -s rtl8723d_config.bin $(1)/lib/firmware/rtl_bt/rtl8723ds_config.bin
endef
$(eval $(call BuildPackage,rtl8723ds_bt-firmware))

Package/rtl8761a-firmware = $(call Package/firmware-default,RealTek RTL8761A firmware)
define Package/rtl8761a-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtl_bt
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtl_bt/rtl8761a_fw.bin $(1)/lib/firmware/rtl_bt
endef
$(eval $(call BuildPackage,rtl8761a-firmware))

Package/rtl8761b-firmware = $(call Package/firmware-default,RealTek RTL8761B firmware)
define Package/rtl8761b-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtl_bt
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtl_bt/rtl8761b_config.bin $(1)/lib/firmware/rtl_bt
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtl_bt/rtl8761b_fw.bin $(1)/lib/firmware/rtl_bt
endef
$(eval $(call BuildPackage,rtl8761b-firmware))

Package/rtl8761bu-firmware = $(call Package/firmware-default,RealTek RTL8761BU firmware)
define Package/rtl8761bu-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtl_bt
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtl_bt/rtl8761bu_config.bin $(1)/lib/firmware/rtl_bt
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtl_bt/rtl8761bu_fw.bin $(1)/lib/firmware/rtl_bt
endef
$(eval $(call BuildPackage,rtl8761bu-firmware))

Package/realtek-bluetooth-firmware = $(call Package/firmware-default,RealTek bluetooth firmware)
define Package/realtek-bluetooth-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtl_bt
	$(CP) \
		$(PKG_BUILD_DIR)/rtl_bt/* \
		$(1)/lib/firmware/rtl_bt
endef
$(eval $(call BuildPackage,realtek-bluetooth-firmware))


Package/rtl8821ae-firmware = $(call Package/firmware-default,RealTek RTL8821AE firmware)
define Package/rtl8821ae-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8821aefw.bin $(1)/lib/firmware/rtlwifi
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtlwifi/rtl8821aefw_wowlan.bin $(1)/lib/firmware/rtlwifi
endef
$(eval $(call BuildPackage,rtl8821ae-firmware))

Package/rtl8822be-firmware = $(call Package/firmware-default,RealTek RTL8822BE firmware)
define Package/rtl8822be-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtw88
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtw88/rtw8822b_fw.bin $(1)/lib/firmware/rtw88
endef
$(eval $(call BuildPackage,rtl8822be-firmware))

Package/rtl8822ce-firmware = $(call Package/firmware-default,RealTek RTL8822CE firmware)
define Package/rtl8822ce-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtw88
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtw88/rtw8822c_fw.bin $(1)/lib/firmware/rtw88
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/rtw88/rtw8822c_wow_fw.bin $(1)/lib/firmware/rtw88
endef
$(eval $(call BuildPackage,rtl8822ce-firmware))

Package/rtw89-firmware = $(call Package/firmware-default,RealTek RTW89 firmware)
define Package/rtw89-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/rtw89
	$(CP) \
		$(PKG_BUILD_DIR)/rtw89/* \
		$(1)/lib/firmware/rtw89
endef
$(eval $(call BuildPackage,rtw89-firmware))
