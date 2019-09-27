##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16 
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

# Imported source files
CHIBIOS = ../third_party/ChibiOS
TFINCLUDE = ./third_party
TF = ./tensorflow
include $(CHIBIOS)/boards/ATMEL_SAM3S_CREATOR/board.mk
include $(CHIBIOS)/os/hal/platforms/AT91SAM3S/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/AT91SAM3S/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include $(CHIBIOS)/test/test.mk



# Define linker script file here
LDSCRIPT= $(PORTLD)/AT91SAM3S2C.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/syscalls.c \
       $(CHIBIOS)/os/various/chprintf.c \
       $(TF)/lite/experimental/micro/tools/make/downloads/kissfft/kiss_fft.c \
       $(TF)/lite/experimental/micro/tools/make/downloads/kissfft/tools/kiss_fftr.c \
       $(TF)/lite/c/c_api_internal.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
#
CPPSRC = i2c.cc \
         hts221.cc \
         lsm9ds1.cc \
         mpl3115a2.cc \
         veml6070.cc \
	 $(TF)/lite/experimental/micro/micro_error_reporter.cc \
	 $(TF)/lite/experimental/micro/micro_mutable_op_resolver.cc \
	 $(TF)/lite/experimental/micro/simple_tensor_allocator.cc \
	 $(TF)/lite/experimental/micro/micro_allocator.cc \
	 $(TF)/lite/experimental/micro/debug_log.cc \
	 $(TF)/lite/experimental/micro/debug_log_numbers.cc \
	 $(TF)/lite/experimental/micro/micro_interpreter.cc \
	 $(TF)/lite/experimental/micro/kernels/depthwise_conv.cc \
	 $(TF)/lite/experimental/micro/kernels/pooling.cc \
	 $(TF)/lite/experimental/micro/kernels/softmax.cc \
	 $(TF)/lite/experimental/micro/kernels/conv.cc \
	 $(TF)/lite/experimental/micro/kernels/elementwise.cc \
	 $(TF)/lite/experimental/micro/kernels/all_ops_resolver.cc \
	 $(TF)/lite/experimental/micro/kernels/fully_connected.cc \
	 $(TF)/lite/core/api/error_reporter.cc \
	 $(TF)/lite/core/api/flatbuffer_conversions.cc \
	 $(TF)/lite/core/api/op_resolver.cc \
	 $(TF)/lite/kernels/kernel_util.cc \
	 $(TF)/lite/kernels/internal/quantization_util.cc  \
	 $(TF)/lite/experimental/micro/examples/micro_speech/audio_provider.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/feature_provider.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/no_micro_features_data.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/yes_micro_features_data.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/tiny_conv_micro_features_model_data.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/recognize_commands.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/command_responder.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/micro_features_generator.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/micro_model_settings.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/fft.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/fft_util.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/filterbank.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/filterbank_util.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/frontend.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/frontend_util.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/log_lut.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/log_scale.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/log_scale_util.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/noise_reduction.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/noise_reduction_util.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/pcan_gain_control.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/pcan_gain_control_util.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/window.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/micro_features/window_util.cc \
	 $(TF)/lite/experimental/micro/examples/micro_speech/main.cc
 
 
# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
	 $(CHIBIOS)/os/various $(TFINCLUDE)/gemmlowp \
	 $(TFINCLUDE)/kissfft/tools $(TFINCLUDE)/kissfft \
	 $(TFINCLUDE)/flatbuffers/include \
         $(TF)/lite/experimental/micro

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m3

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++ -std=c++11
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
#LD   = $(TRGT)gcc
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = -lm

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

include $(CHIBIOS)/os/ports/GCC/ARMCMx/rules.mk
