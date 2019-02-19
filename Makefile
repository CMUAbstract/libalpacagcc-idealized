LIB = libalpacagcc

export LIBALPACAGCC_ENABLE_DIAGNOSTICS = 0
export LIBALPACAGCC_CONF_REPORT = 0

OBJECTS += \
  alpaca.o

DEPS += \
	libmsp \

override SRC_ROOT = ../../src

override CFLAGS += \
	-I$(SRC_ROOT)/include \
	-I$(SRC_ROOT)/include/$(LIB) \

include $(MAKER_ROOT)/Makefile.$(TOOLCHAIN)
