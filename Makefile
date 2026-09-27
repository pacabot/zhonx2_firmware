CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

BUILD ?= build
TARGET := $(BUILD)/ZHONX_II_M4
OPT ?= -O2
DEBUG_FLAGS ?= -g3

# Source set from the original ZHONX_II_M4 Xcode project.
SOURCES := $(shell python3 tools/project_sources.py)
SOURCES += pacabot/src/app/menu_colin.c pacabot/src/app/bezier_curves.c \
 pacabot/src/app/solverMaze/robotInterface.c \
 pacabot/src/app/solverMaze/solverMaze.c pacabot/src/app/solverMaze/run.c \
 pacabot/src/app/solverMaze/user_interface.c
SOURCES += $(wildcard firmware/core/*.c firmware/platform/*.c)
OBJECTS := $(addprefix $(BUILD)/,$(SOURCES:.c=.o))
DEPS := $(OBJECTS:.o=.d)

CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFINES := -DSTM32F405RG -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -D__FPU_USED
INCLUDES := -Ifirmware/include -I. -Icmsis -Icmsis_boot -Icmsis_lib/include -Ipacabot -Ipacabot/include \
 -Ipacabot/include/app -Ipacabot/include/config -Ipacabot/include/oled \
 -Ipacabot/include/hal -Ipacabot/include/drivers -Ipacabot/include/util -Ipacabot/src
CFLAGS := $(CPUFLAGS) $(DEFINES) $(INCLUDES) -std=gnu11 $(OPT) $(DEBUG_FLAGS) -fcommon \
 -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS := $(CPUFLAGS) -Tpacabot_link.ld -Wl,--gc-sections,-Map=$(TARGET).map \
 --specs=nano.specs -lm

.PHONY: all app clean boot test package
all: package
app: $(TARGET).elf $(TARGET).bin $(TARGET).hex $(BUILD)/application.ota.bin

$(BUILD)/firmware/%.o: CFLAGS += -Wall -Wextra -Werror

$(BUILD)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS) pacabot_link.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/application.ota.bin: $(TARGET).bin tools/fw_package.py
	python3 tools/fw_package.py --app $(BUILD)

clean:
	rm -rf $(BUILD)

-include $(DEPS)

BOOT_SOURCES := firmware/boot/startup.c firmware/boot/boot.c firmware/core/fw_crc.c \
 firmware/core/fw_update.c firmware/core/fw_protocol.c firmware/platform/stm32_flash.c \
 cmsis_boot/system_stm32f4xx.c cmsis_lib/source/stm32f4xx_flash.c
boot: $(BUILD)/bootloader.bin
$(BUILD)/bootloader.elf: Makefile $(BOOT_SOURCES) firmware/boot/boot.ld $(wildcard firmware/include/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -Wall -Wextra -Werror -nostartfiles $(BOOT_SOURCES) -Tfirmware/boot/boot.ld \
	 -Wl,--gc-sections,-Map=$(BUILD)/bootloader.map --specs=nano.specs -o $@
	$(SIZE) $@
$(BUILD)/bootloader.bin: $(BUILD)/bootloader.elf
	$(OBJCOPY) -O binary $< $@
package: app boot
	python3 tools/fw_package.py $(BUILD)
test:
	@mkdir -p $(BUILD)
	cc -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
	 -Ifirmware/include tests/test_firmware.c firmware/core/*.c -o $(BUILD)/test_firmware
	$(BUILD)/test_firmware
	cc -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
	 -Ifirmware/include tests/test_library.c firmware/core/fw_library.c firmware/core/nimes.c -o $(BUILD)/test_library
	$(BUILD)/test_library
	cc -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
	 -Itests/mocks -Ifirmware/include -Ipacabot/include tests/test_menu.c firmware/platform/fw_menu.c firmware/core/fw_calibration.c -o $(BUILD)/test_menu
	$(BUILD)/test_menu
	cc -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
	 -Ifirmware/include tests/test_calibration.c firmware/core/fw_calibration.c -o $(BUILD)/test_calibration
	$(BUILD)/test_calibration
	cc -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
	 -Ifirmware/include tests/test_calibration_extra.c firmware/core/fw_calibration.c firmware/core/fw_cal_extra.c -o $(BUILD)/test_calibration_extra
	$(BUILD)/test_calibration_extra
	cc -std=gnu11 -O1 -g -fcommon -Wall -Wextra -Werror -fsanitize=address,undefined \
	 -Itests/mocks -Ifirmware/include -Ipacabot/include tests/test_motion.c \
	 firmware/platform/fw_motion.c firmware/core/wall_control.c firmware/core/fw_calibration.c firmware/core/fw_cal_extra.c -o $(BUILD)/test_motion
	$(BUILD)/test_motion
	cc -std=gnu11 -O1 -g -fcommon -Wall -Wextra -Werror -fsanitize=address,undefined \
	 -Itests/mocks -Ifirmware/include -Ipacabot/include tests/test_navigation.c \
	 firmware/platform/fw_app.c firmware/core/fw_library.c firmware/core/nimes.c firmware/core/fw_start.c firmware/core/fw_calibration.c firmware/core/fw_cal_extra.c -o $(BUILD)/test_navigation
	$(BUILD)/test_navigation
	cc -std=gnu11 -O1 -g -fsanitize=address,undefined \
	 -Itests/mocks -Ifirmware/include -Ipacabot/include tests/test_ui.c \
	 firmware/platform/fw_ui.c firmware/platform/fw_menu_view.c firmware/core/fw_library.c firmware/core/nimes.c firmware/core/fw_text.c \
	 pacabot/src/oled/ssd1306.c pacabot/src/oled/smallfonts.c -o $(BUILD)/test_ui
	$(BUILD)/test_ui
	cc -std=gnu11 -O1 -g -fsanitize=address,undefined \
	 -Itests/mocks -Ifirmware/include -Ipacabot/include tests/test_calibration_ui.c \
	 firmware/platform/fw_calibration_ui.c firmware/core/fw_calibration.c firmware/core/fw_cal_extra.c firmware/core/fw_text.c \
	 pacabot/src/oled/ssd1306.c pacabot/src/oled/smallfonts.c -o $(BUILD)/test_calibration_ui
	$(BUILD)/test_calibration_ui
	cc -std=gnu11 -O1 -g -fsanitize=address,undefined \
	 -Itests/mocks -Ifirmware/include -Ipacabot/include tests/test_hardware.c \
	 firmware/platform/fw_hardware.c firmware/core/fw_text.c pacabot/src/oled/ssd1306.c pacabot/src/oled/smallfonts.c -o $(BUILD)/test_hardware
	$(BUILD)/test_hardware
	python3 -m unittest discover -s tests -p 'test_*.py'
