catalog = make/sources.txt

USE_USBH  = 1
USE_FATFS = 1

USER_INCLUDES += -Isrc/4spoor -Iext/ct-synstack/src

CFLAGS += -O2 -ffast-math -g