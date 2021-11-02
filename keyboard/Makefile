OUTPUT_DIR = _build
SUBDIRS=$(filter-out $(OUTPUT_DIR),$(shell ls -l | grep ^d | grep -v 'ergoconn\|g84-4100\|lkb-core\|lot60-ble\|template\|test' | awk '{print $$9}'))
export VERSION=af$(shell git log --abbrev-commit --pretty=oneline -1 | cut -c 1-6 || echo 'unknown')
export BUILDTIME=$(shell date +"%Y%m%d")
export FILE_VERSION = $(BUILDTIME)-$(VERSION)
export RELEASE_DIR = ../$(OUTPUT_DIR)

.PHONY: clean whole dist_clean help
whole: $(SUBDIRS)

$(SUBDIRS): $(OUTPUT_DIR)
	@-mkdir $(OUTPUT_DIR)/$@
	@make -C $@ package OUTPUT_DIRECTORY=$(RELEASE_DIR)/$@ NRF_PACKAGE_NAME=$(RELEASE_DIR)/$@-nrf52_dfu-$(FILE_VERSION).zip BUILD_PREFIX=$@
	@make -C $@ usb OUTPUT_DIRECTORY=$(RELEASE_DIR)/$@ OUTPUT_BIN=$(RELEASE_DIR)/$@-ch554-$(FILE_VERSION).bin BUILD_PREFIX=$@
	@make -C $@ merge_setting OUTPUT_DIRECTORY=$(RELEASE_DIR)/$@ NRF_MERGE_SIGN_NAME=$(RELEASE_DIR)/$@-nrf52_kbd_sign-$(FILE_VERSION).hex BUILD_PREFIX=$@
	@make -C $@ all OUTPUT_DIRECTORY=$(RELEASE_DIR)/$@ NRF_MERGE_ALL_NAME=$(RELEASE_DIR)/$@-nrf52_all-$(FILE_VERSION).hex BUILD_PREFIX=$@
	
$(OUTPUT_DIR):
	@-mkdir -p $(OUTPUT_DIR)

dist_clean: 
	-rm -r $(OUTPUT_DIR)/*/

clean:
	-rm -rf $(OUTPUT_DIR)

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52_kbd           - default： build KBD firmware
	@echo		setting             - generate dfu setting
	@echo		bootloader          - build BootLoader firmware
	@echo		package             - pack firmware for DFU
	@echo		all                 - merge bootloader softdevice and dfu setting with application
	@echo		erase               - erase the chip
	@echo		merge_setting       - merge dfu setting with application
	@echo		merge_softdevice    - merge softdevice with application
	@echo		flash               - Flashing KBD firmware
	@echo		flash_setting       - Flashing KBD firmware and dfu setting
	@echo		flash_bootloader    - Flashing bootloader
	@echo		flash_softdevice    - Flashing softdevice
	@echo		flash_all           - Flashing bootloader softdevice and application with dfu setting
	@echo		whole               - make all keyboards firmware in the keyboard folder
	@echo All targets starts with "flash" could has prefix "pyocd_" or "openocd", which \
	means use pyocd  or openocd to flash chip. 
