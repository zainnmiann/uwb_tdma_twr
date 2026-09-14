# ---------------------------------------------------------------------------
# uwb_tdma_twr — nRF52 (DWM1001-DEV) target.
#
# Builds core/ + port/nrf52/. Never touches port/esp32/ or sim/ -- those are
# built by platformio.ini and sim/Makefile respectively.
#
#   make            build _build/uwb_tdma_twr.hex
#   make flash      program over J-Link (nrfjprog)
#   make flash-jl   program over J-Link (JLinkExe, no nrfjprog needed)
#   make size       section sizes
#   make clean
#   make lint-core  fail if core/ has picked up any platform dependency
#
# Override the toolchain prefix if it is not on PATH, e.g.
#   make GNU_INSTALL_ROOT=/opt/arm-gnu-toolchain-13.2/bin/
# ---------------------------------------------------------------------------

PROJECT_NAME := uwb_tdma_twr
OUTPUT_DIR   := _build

ARM_FALLBACK_BIN := $(firstword $(wildcard $(HOME)/opt/arm-gnu-toolchain-*/bin))
GNU_USABLE := $(shell echo '\#include <stdint.h>' | arm-none-eabi-gcc -E - > /dev/null 2>&1 && echo yes)

ifneq ($(GNU_USABLE),yes)
ifneq ($(ARM_FALLBACK_BIN),)
GNU_INSTALL_ROOT ?= $(ARM_FALLBACK_BIN)/
endif
endif

GNU_INSTALL_ROOT ?=
GNU_PREFIX       ?= arm-none-eabi

CC      := $(GNU_INSTALL_ROOT)$(GNU_PREFIX)-gcc
AS      := $(GNU_INSTALL_ROOT)$(GNU_PREFIX)-gcc
LD      := $(GNU_INSTALL_ROOT)$(GNU_PREFIX)-gcc
OBJCOPY := $(GNU_INSTALL_ROOT)$(GNU_PREFIX)-objcopy
SIZE    := $(GNU_INSTALL_ROOT)$(GNU_PREFIX)-size

NRF52   := port/nrf52
SDK     := $(NRF52)/vendor/nRF5_SDK_14.2.0
DECA    := $(NRF52)/vendor/deca_driver
BOARDS  := $(NRF52)/vendor/boards

# --- sources -----------------------------------------------------------
# The portable protocol core -- identical set to the sim/ and platformio.ini
# builds. Never add a platform include to any of these; make lint-core checks.
CORE_C := \
  core/app.c \
  core/cfg.c \
  core/timing.c \
  core/slots.c \
  core/ranging.c \
  core/sync.c \
  core/role_initiator.c \
  core/role_responder.c \
  core/output.c \
  core/serial.c

PORT_C := \
  $(NRF52)/main.c \
  $(NRF52)/radio.c \
  $(NRF52)/uart_io.c \
  $(NRF52)/port_persist.c \
  $(NRF52)/port_sys.c \
  $(NRF52)/board_defaults.c \
  $(NRF52)/syscalls.c

VENDOR_C := \
  $(DECA)/deca_device.c \
  $(DECA)/deca_params_init.c \
  $(DECA)/deca_range_tables.c \
  $(DECA)/port/port_platform.c \
  $(SDK)/components/boards/boards.c \
  $(SDK)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK)/components/drivers_nrf/spi_master/nrf_drv_spi.c \
  $(SDK)/components/drivers_nrf/hal/nrf_nvmc.c \
  $(SDK)/components/libraries/util/app_error.c \
  $(SDK)/components/libraries/util/app_error_weak.c \
  $(SDK)/components/libraries/util/app_util_platform.c \
  $(SDK)/components/libraries/util/nrf_assert.c \
  $(SDK)/components/toolchain/system_nrf52.c

SRC_C := $(CORE_C) $(PORT_C) $(VENDOR_C)

SRC_S := \
  $(SDK)/components/toolchain/gcc/gcc_startup_nrf52.S

INC_DIRS := \
  core \
  port \
  $(NRF52) \
  $(NRF52)/config \
  $(BOARDS) \
  $(DECA) \
  $(DECA)/port \
  $(SDK)/components/boards \
  $(SDK)/components/device \
  $(SDK)/components/drivers_nrf/common \
  $(SDK)/components/drivers_nrf/delay \
  $(SDK)/components/drivers_nrf/hal \
  $(SDK)/components/drivers_nrf/nrf_soc_nosd \
  $(SDK)/components/drivers_nrf/spi_master \
  $(SDK)/components/drivers_nrf/uart \
  $(SDK)/components/libraries/experimental_section_vars \
  $(SDK)/components/libraries/experimental_log \
  $(SDK)/components/libraries/experimental_log/src \
  $(SDK)/components/libraries/experimental_memobj \
  $(SDK)/components/libraries/balloc \
  $(SDK)/components/libraries/atomic \
  $(SDK)/components/libraries/strerror \
  $(SDK)/components/libraries/util \
  $(SDK)/components/toolchain \
  $(SDK)/components/toolchain/cmsis/include \
  $(SDK)/components/toolchain/gcc \
  $(SDK)/external/fprintf

# --- flags -----------------------------------------------------------------

CPU_FLAGS := -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16

DEFINES := \
  -DBOARD_DW1001_DEV \
  -DBSP_DEFINES_ONLY \
  -DCONFIG_GPIO_AS_PINRESET \
  -DFLOAT_ABI_HARD \
  -DNRF52 \
  -DNRF52832_XXAA \
  -DNRF52_PAN_74 \
  -DSWI_DISABLE0

OPT := -O2 -g3

CFLAGS := $(CPU_FLAGS) $(DEFINES) $(OPT) \
  -std=gnu11 \
  -Wall -Wextra -Wno-unused-parameter -Wno-expansion-to-defined \
  -ffunction-sections -fdata-sections -fno-strict-aliasing \
  -fno-builtin -fshort-enums \
  $(addprefix -I,$(INC_DIRS))

# The vendored Decawave/Nordic sources are kept byte-for-byte identical to
# upstream, so their warnings are silenced rather than fixed.
VENDOR_CFLAGS := -Wno-all -w

ASMFLAGS := $(CPU_FLAGS) $(DEFINES) -g3 -x assembler-with-cpp \
  -D__HEAP_SIZE=2048 -D__STACK_SIZE=8192 \
  $(addprefix -I,$(INC_DIRS))

LDSCRIPT := $(SDK)/components/toolchain/gcc/nrf52_xxaa.ld

LDFLAGS := $(CPU_FLAGS) \
  -T$(LDSCRIPT) -L$(SDK)/components/toolchain/gcc \
  -Wl,--gc-sections -Wl,-Map=$(OUTPUT_DIR)/$(PROJECT_NAME).map \
  --specs=nano.specs --specs=nosys.specs

LIBS := -lm

# --- objects -----------------------------------------------------------

OBJ := $(addprefix $(OUTPUT_DIR)/,$(SRC_C:.c=.o)) \
       $(addprefix $(OUTPUT_DIR)/,$(SRC_S:.S=.o))

ELF := $(OUTPUT_DIR)/$(PROJECT_NAME).elf
HEX := $(OUTPUT_DIR)/$(PROJECT_NAME).hex
BIN := $(OUTPUT_DIR)/$(PROJECT_NAME).bin

.PHONY: all clean size flash flash-jl erase check-driver lint-core lint-externc

all: $(HEX) $(BIN) size

$(OUTPUT_DIR)/$(DECA)/%.o: $(DECA)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(VENDOR_CFLAGS) -c $< -o $@

$(OUTPUT_DIR)/$(SDK)/%.o: $(SDK)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(VENDOR_CFLAGS) -c $< -o $@

$(OUTPUT_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(OUTPUT_DIR)/%.o: %.S
	@mkdir -p $(dir $@)
	$(AS) $(ASMFLAGS) -c $< -o $@

$(ELF): $(OBJ)
	@mkdir -p $(dir $@)
	$(LD) $(LDFLAGS) $(OBJ) $(LIBS) -o $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

size: $(ELF)
	@$(SIZE) $(ELF)

clean:
	rm -rf $(OUTPUT_DIR)

# Fails if anything under port/nrf52/vendor/deca_driver has drifted from
# upstream Decawave (the same check the reference DWM1001 tree carries).
check-driver:
	@shasum -a 256 -c $(NRF52)/vendor/deca_driver.sha256 >/dev/null && \
	  echo "deca_driver matches upstream Decawave/dwm1001-examples"

# The porting boundary is core/ + port/<target>/*.h -- core/ must never pick
# up a platform header, a driver call, or a framework include.
lint-core:
	@bad=0; \
	for pat in "nrf\.h" "nrf_nvmc" "Arduino\.h" "deca_device_api\.h" "boards\.h" "display\.h" "esp32" "Preferences\.h"; do \
	  hits=$$(grep -lE "$$pat" core/*.c core/*.h 2>/dev/null || true); \
	  if [ -n "$$hits" ]; then echo "lint-core: '$$pat' found in: $$hits"; bad=1; fi; \
	done; \
	[ $$bad -eq 0 ] && echo "lint-core OK (core/ has no platform leaks)" || exit 1

# Every header in core/ and port/ must guard with extern "C" and must not
# #include anything from inside that guard (the bug this project's design
# doc calls out: an include inside extern "C" silently gets the wrong
# linkage depending on which .cpp reaches it first).
lint-externc:
	@fail=0; \
	for f in core/*.h port/*.h; do \
	  grep -q '__cplusplus' "$$f" || { echo "$$f: missing extern \"C\" guard"; fail=1; }; \
	  awk '/extern "C"/{in_blk=1; next} /^\}/{in_blk=0} in_blk && /^[ \t]*#[ \t]*include/{print FILENAME": include inside extern \"C\" block"; e=1} END{exit e+0}' "$$f" || fail=1; \
	done; \
	[ $$fail -eq 0 ] && echo "lint-externc OK" || exit 1

flash: $(HEX)
	nrfjprog -f nrf52 --program $(HEX) --sectorerase --verify
	nrfjprog -f nrf52 --reset

flash-jl: $(HEX)
	@printf 'device NRF52832_XXAA\nsi 1\nspeed 4000\nr\nh\nloadfile %s\nr\ng\nqc\n' \
	  "$(abspath $(HEX))" > $(OUTPUT_DIR)/flash.jlink
	JLinkExe -CommanderScript $(OUTPUT_DIR)/flash.jlink

erase:
	nrfjprog -f nrf52 --eraseall

-include $(shell find $(OUTPUT_DIR) -name '*.d' 2>/dev/null)
