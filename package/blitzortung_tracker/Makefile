
include $(TOPDIR)/rules.mk

# Make stuff work with downloaded SDK+toolchain:
TARGET_LDFLAGS+=-rpath-link $(shell find $(STAGING_DIR) -name ld-uClibc.so.0)

PKG_NAME:=blitzortung_tracker
PKG_VERSION:=24
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/blitzortung_tracker
	SECTION:=net
	CATEGORY:=Network
	TITLE:=Blitzortung.org Tracker Program
	URL:=http://www.blitzortung.org/
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
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/tracker_Linux $(1)/bin/blitzortung_tracker
endef

$(eval $(call BuildPackage,blitzortung_tracker))

