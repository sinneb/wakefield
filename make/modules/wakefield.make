catalog = make/sources.txt

USE_USBH  = 0
USE_FATFS = 1

USER_INCLUDES += -Isrc/wakefield

CFLAGS += -O0 -ffast-math -g
