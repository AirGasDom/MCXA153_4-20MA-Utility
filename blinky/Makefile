TARGET  := blinky
BUILD   := build
CROSS   := arm-none-eabi-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size
PYOCD   ?= $(HOME)/.venvs/pyocd/bin/pyocd

SRCS := main.c settings.c device/system_MCXA153.c device/startup_MCXA153.S
INCS := -I. -Idevice -Idevice/periph -Idevice/drivers -Icmsis
LDSCRIPT := device/MCXA153_flash.ld

CPU    := -mcpu=cortex-m33+nodsp -mthumb -mfloat-abi=soft
DEFS   := -DCPU_MCXA153VLH -D__STARTUP_CLEAR_BSS -D__STARTUP_INITIALIZE_RAMFUNCTION
CFLAGS := $(CPU) $(DEFS) $(INCS) -Os -g3 -Wall -ffunction-sections -fdata-sections -std=gnu11
LDFLAGS := $(CPU) -T$(LDSCRIPT) -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map \
           --specs=nano.specs --specs=nosys.specs

OBJS := $(addprefix $(BUILD)/,$(addsuffix .o,$(basename $(SRCS))))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin
	$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

flash: $(BUILD)/$(TARGET).elf
	$(PYOCD) flash -t mcxa153vlh $<

clean:
	rm -rf $(BUILD)

.PHONY: all flash clean
