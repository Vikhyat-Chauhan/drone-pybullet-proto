# Cross-compiles the gem5 benchmark harness (main.c) statically linked
# against the real APE planner sources (native/ape_ops/src/) and gem5's
# m5ops magic-instruction shim (util/m5/src/abi/arm/m5op.S), targeting
# the single CPU this whole repo models: a Cortex-M7-APPROXIMATION
# (gem5 has no true M-profile CPU model; this is deliberately a standard
# ARM/glibc SE-mode binary run under an in-order MinorCPU config,
# matching the approximation ../configs/cortex_m7_inorder.py documents)
# — representative of a low-compute drone's flight-controller-class MCU.
#
# Requires GEM5_ROOT (path to a gem5 checkout with `include/gem5/m5ops.h`
# and `util/m5/src/abi/arm/m5op.S`) and arm-linux-gnueabihf-gcc.
#
# Usage:
#   GEM5_ROOT=/path/to/gem5 make arm

GEM5_ROOT       ?= $(error GEM5_ROOT must be set to a gem5 checkout)
APE_SRC_DIR     := ../../src
APE_INC_DIR     := ../../include
BUILD_DIR       := build

ARM_CC          ?= arm-linux-gnueabihf-gcc

COMMON_SRCS     := src/main.c $(APE_SRC_DIR)/ape1_bug.c $(APE_SRC_DIR)/ape2_dwa.c \
                    $(APE_SRC_DIR)/ape3_vfh.c $(APE_SRC_DIR)/ape_common.c
COMMON_CFLAGS   := -O2 -static -I$(APE_INC_DIR) -I$(GEM5_ROOT)/include

ARM_M5OP        := $(GEM5_ROOT)/util/m5/src/abi/arm/m5op.S

.PHONY: arm clean

arm: $(BUILD_DIR)/arm/bench_ape1 $(BUILD_DIR)/arm/bench_ape2 $(BUILD_DIR)/arm/bench_ape3

$(BUILD_DIR)/arm/bench_ape%: $(COMMON_SRCS) $(ARM_M5OP) $(APE_INC_DIR)/*.h | $(BUILD_DIR)/arm
	# -marm: this toolchain defaults to Thumb with interworking disabled,
	# which corrupts the return address across the call into m5op.S's
	# plain-ARM-mode magic-instruction stubs (observed as a page fault at
	# a near-null PC under gem5's MinorCPU). Force the whole binary to
	# ARM (A32) mode to avoid the ARM/Thumb interworking boundary entirely.
	$(ARM_CC) $(COMMON_CFLAGS) -marm -DAPE_BENCH_TARGET=$* $(COMMON_SRCS) $(ARM_M5OP) -o $@ -lm

$(BUILD_DIR)/arm:
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)
