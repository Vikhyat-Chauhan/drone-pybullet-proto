# Builds libape_ops.so for the HOST architecture (whatever `cc` targets
# on this machine — x86_64, native ARM64, etc.). Not cross-compiled, no
# -static. This is what the live navigator loads via ctypes
# (ape_native.py, repo root) to run genuine APE compute instead
# of a time.sleep() stand-in.
#
# For the gem5 cross-compiled benchmark (offline hardware-timing study,
# not used by the live navigator), see gem5_bench/bench/Makefile —
# cross-compiles the same planner sources for arm-linux-gnueabihf
# (Cortex-M7-approximation study, the single CPU this repo models) — an
# independent build target, different toolchain, does not depend on this
# Makefile or vice versa.
#
# NOTE: libape_ops.so is architecture-specific. If this repo is deployed
# to different hardware, rerun `make native` on the target machine — the
# .so is not portable.

CC        ?= cc
BUILD_DIR := build
CFLAGS    := -O2 -fPIC -Wall -Wextra -Iinclude
SRCS      := src/native_api.c src/ape1_bug.c src/ape2_dwa.c src/ape3_vfh.c src/ape_common.c
LIB       := $(BUILD_DIR)/libape_ops.so

.PHONY: native clean
native: $(LIB)

$(LIB): $(SRCS) include/*.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -shared $(SRCS) -o $@ -lm

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
