catalog = make/sources.txt

USE_USBH  = 1
USE_FATFS = 1

USER_INCLUDES += -Isrc/4spoor -Iext/ct-synstack/src

CFLAGS += -O3 -ffast-math -g
