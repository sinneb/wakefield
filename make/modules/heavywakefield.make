catalog = make/sources.txt

USE_USBH  = 0
USE_FATFS = 0

USER_INCLUDES += -Isrc/heavywakefield

CFLAGS += -O3 -g -ffast-math -DNDEBUG
CPPFLAGS += -O3 -g -ffast-math -DNDEBUG
