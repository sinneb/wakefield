catalog = make/sources.txt

USE_USBH  = 0
USE_FATFS = 1

USER_INCLUDES += -Isrc/octapal

CFLAGS += -O3 -ffast-math -g
