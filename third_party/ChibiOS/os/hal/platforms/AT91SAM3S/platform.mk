# List of all the AT91SAM3 platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/platforms/AT91SAM3S/hal_lld.c \
              ${CHIBIOS}/os/hal/platforms/AT91SAM3S/system_SAM3.c \
              ${CHIBIOS}/os/hal/platforms/AT91SAM3S/pal_lld.c \
              ${CHIBIOS}/os/hal/platforms/AT91SAM3S/serial_lld.c \
              ${CHIBIOS}/os/hal/platforms/AT91SAM3S/atmel_adc.c \
              ${CHIBIOS}/os/hal/platforms/AT91SAM3S/pio.c \
              ${CHIBIOS}/os/hal/platforms/AT91SAM3S/pmc.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/platforms/AT91SAM3S
