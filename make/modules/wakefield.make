catalog = make/sources.txt

USE_USBH  = 0
USE_FATFS = 0

USER_INCLUDES += -Isrc/wakefield

CFLAGS += -O3 -ffast-math -g
