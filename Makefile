CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

BUILD := build
TARGET := $(BUILD)/ZHONX_II_M4

# Source set from the original ZHONX_II_M4 Xcode project.
SOURCES := $(shell python3 tools/project_sources.py)
SOURCES += pacabot/src/app/menu_colin.c pacabot/src/app/bezier_curves.c \
 pacabot/src/app/solverMaze/robotInterface.c
OBJECTS := $(addprefix $(BUILD)/,$(SOURCES:.c=.o))
DEPS := $(OBJECTS:.o=.d)

CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFINES := -DSTM32F405RG -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -D__FPU_USED
INCLUDES := -I. -Icmsis -Icmsis_boot -Icmsis_lib/include -Ipacabot -Ipacabot/include \
 -Ipacabot/include/app -Ipacabot/include/config -Ipacabot/include/oled \
 -Ipacabot/include/hal -Ipacabot/include/drivers -Ipacabot/include/util -Ipacabot/src
CFLAGS := $(CPUFLAGS) $(DEFINES) $(INCLUDES) -std=gnu11 -O0 -g3 -fcommon \
 -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS := $(CPUFLAGS) -Tpacabot_link.ld -Wl,--gc-sections,-Map=$(TARGET).map \
 --specs=nano.specs -lm

.PHONY: all clean
all: $(TARGET).elf $(TARGET).bin $(TARGET).hex

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS) pacabot_link.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(BUILD)

-include $(DEPS)
