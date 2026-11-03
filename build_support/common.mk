.DEFAULT_GOAL := all
BUILD_DIR := build
SUPPORT := ../build_support

PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size
PROGRAMMER ?= STM32_Programmer_CLI

CPU := -mcpu=cortex-m4
FPU := -mfpu=fpv4-sp-d16
FLOAT_ABI := -mfloat-abi=hard
MCU := $(CPU) -mthumb $(FPU) $(FLOAT_ABI)
OPT ?= -O2
CFLAGS := $(MCU) $(DEFINES) $(INCLUDES) $(OPT) -g3 -std=gnu11 \
	-ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -MMD -MP
ASFLAGS := $(MCU) -x assembler-with-cpp -MMD -MP
LINKER_SCRIPT := $(SUPPORT)/STM32F407IGHx_FLASH.ld
LDFLAGS := $(MCU) -T$(LINKER_SCRIPT) \
	-Wl,-Map=$(BUILD_DIR)/$(PROJECT).map,--cref,--gc-sections \
	-Wl,--defsym=_Min_Stack_Size=$(STACK_SIZE),--defsym=_Min_Heap_Size=$(HEAP_SIZE) \
	--specs=nano.specs --specs=nosys.specs
LIBS := -lc -lm

OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/shared/,$(COMMON_C_SOURCES:.c=.o))
OBJECTS += $(BUILD_DIR)/shared/startup_stm32f407xx.o
OBJECTS += $(BUILD_DIR)/shared/sysmem.o
DEPS := $(OBJECTS:.o=.d)
ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin
HEX := $(BUILD_DIR)/$(PROJECT).hex

# MSYS Make uses /bin/sh; native Windows Make needs cmd.exe.
NATIVE_WINDOWS := $(if $(filter Windows_NT,$(OS)),$(if $(filter /%,$(SHELL)),,yes))
ifeq ($(NATIVE_WINDOWS),yes)
SHELL := cmd.exe
.SHELLFLAGS := /c
MKDIR = if not exist "$(subst /,\,$(@D))" mkdir "$(subst /,\,$(@D))"
else
MKDIR = mkdir -p "$(@D)"
endif

.PHONY: all clean size probe flash
all: $(ELF) $(BIN) $(HEX)

$(ELF): $(OBJECTS) $(PREBUILT_LIBS) $(LINKER_SCRIPT) Makefile $(SUPPORT)/common.mk
	@echo Linking $@
	@"$(CC)" $(OBJECTS) $(LDFLAGS) -Wl,--start-group $(PREBUILT_LIBS) $(LIBS) -Wl,--end-group -o $@
	@"$(SIZE)" $@

$(BIN): $(ELF)
	@"$(OBJCOPY)" -O binary $< $@

$(HEX): $(ELF)
	@"$(OBJCOPY)" -O ihex $< $@

$(BUILD_DIR)/%.o: %.c Makefile $(SUPPORT)/common.mk
	@$(MKDIR)
	@echo Compiling $<
	@"$(CC)" $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/shared/%.o: $(SUPPORT)/%.c Makefile $(SUPPORT)/common.mk
	@$(MKDIR)
	@echo Compiling $<
	@"$(CC)" $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/shared/%.o: $(SUPPORT)/%.s Makefile $(SUPPORT)/common.mk
	@$(MKDIR)
	@echo Assembling $<
	@"$(AS)" $(ASFLAGS) -c $< -o $@

size: $(ELF)
	@"$(SIZE)" $<

probe:
	"$(PROGRAMMER)" -l stlink

ifeq ($(CONFIRM_FLASH),yes)
flash: $(BIN)
	"$(PROGRAMMER)" -c port=SWD -w "$(BIN)" 0x08000000 -v -rst
else
flash:
	$(error Flashing replaces firmware and can activate actuators. Disconnect actuators and explicitly pass CONFIRM_FLASH=yes)
endif

clean:
ifneq ($(BUILD_DIR),build)
	$(error Refusing to clean a directory other than build)
else ifeq ($(NATIVE_WINDOWS),yes)
	@if exist build rmdir /s /q build
else
	rm -rf -- build
endif

-include $(DEPS)
