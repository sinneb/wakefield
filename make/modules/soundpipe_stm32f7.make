catalog = make/sources.txt

USE_USBH  = 0
USE_FATFS = 0

USER_INCLUDES += -Isrc/soundpipe_stm32f7

CFLAGS += -O3 -ffast-math -g
