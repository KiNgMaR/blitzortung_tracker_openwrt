
include $(TOPDIR)/rules.mk

PKG_NAME:=blitzortung_tracker
PKG_VERSION:=XLT3
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/blitzortung_tracker
	SECTION:=net
	CATEGORY:=Network
	TITLE:=Blitzortung.org Tracker Program
	URL:=http://www.blitzortung.org/
	DEPENDS:=+libpthread
endef

define Package/blitzortung_tracker/description
	RS232 tracker software for blitzortung.org stations.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Package/blitzortung_tracker/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/blitzortung_tracker $(1)/bin/blitzortung_tracker
endef

$(eval $(call BuildPackage,blitzortung_tracker))

