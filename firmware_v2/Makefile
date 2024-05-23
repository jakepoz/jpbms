BINARY = main

LDSCRIPT = stm32l053r8t6.ld

LIBNAME		= opencm3_stm32l0
DEFS		+= -DSTM32L0

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m0plus $(FP_FLAGS)

# Uncomment For semihosting uart output
#LDFLAGS		+= --specs=rdimon.specs
#LDLIBS		+= -lrdimon

################################################################################
# OpenOCD specific variables

OOCD		?= openocd
OOCD_INTERFACE	?= stlink-v2-1
OOCD_TARGET	?= stm32l0

################################################################################
# Black Magic Probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
BMP_PORT	?= COM6

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242


include rules.mk
