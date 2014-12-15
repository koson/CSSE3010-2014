# put your *.o targets here, make should handle the rest!

STDPERPH_PATH=$(NP2_ROOT)/src/Libraries/STM32F4xx_StdPeriph_Driver
NP2_PATH=$(NP2_ROOT)/src/netduinoplus2
VCP_PATH=$(NP2_ROOT)/src/netduinoplus2/vcp
USBCORE_PATH=$(NP2_ROOT)/src/Libraries/STM32_USB/STM32_USB_Device_Library/Core
USBCDC_PATH=$(NP2_ROOT)/src/Libraries/STM32_USB/STM32_USB_Device_Library/Class/cdc
USBOTG_PATH=$(NP2_ROOT)/src/Libraries/STM32_USB/STM32_USB_OTG_Driver
FREERTOS_PATH=$(NP2_ROOT)/src/FreeRTOS
FATFS_PATH=$(NP2_ROOT)/src/FatFs
CLI_PATH=$(NP2_ROOT)/src/FreeRTOSplus/FreeRTOS_cli
ENC_PATH=$(NP2_ROOT)/src/net/lwip/port/enc28j60
NET_PATH=$(NP2_ROOT)/src/net/lwip
TUIO_PATH=$(NP2_ROOT)/src/net/apps/tuio

PROJ_NAME=main

SRCS = $(PROJ_NAME).c
SRCS += $(NP2_PATH)/net/*.c 
SRCS += $(STDPERPH_PATH)/src/*.c
SRCS += $(NP2_PATH)/src/*.c
SRCS += $(VCP_PATH)/src/*.c $(USBCORE_PATH)/src/*.c $(USBCDC_PATH)/src/usbd_cdc_core.c 
SRCS += $(USBOTG_PATH)/src/usb_dcd.c $(USBOTG_PATH)/src/usb_dcd_int.c $(USBOTG_PATH)/src/usb_core.c
SRCS += $(FREERTOS_PATH)/*.c $(FREERTOS_PATH)/portable/GCC/ARM_CM4F/*.c $(FREERTOS_PATH)/portable/MemMang/heap_1.c
#SRCS += $(FATFS_PATH)/*.c $(NP2_PATH)/mmc/*.c
#SRCS += $(CLI_PATH)/*.c
SRCS += $(ENC_PATH)/*.c #$(ENC_PATH)/netif/*.c
SRCS += $(NET_PATH)/src/core/*.c $(NET_PATH)/src/api/*.c $(NET_PATH)/src/core/ipv4/*.c $(NET_PATH)/src/netif/*.c
SRCS += $(NP2_ROOT)/src/net/apps/tuio/*.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O1 -T$(NP2_PATH)/stm32_flash.ld -Wall -Wmaybe-uninitialized #-Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -DUSE_STDPERIPH_DRIVER -DUSE_STM324xG_EVAL
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

###################################################

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

CFLAGS += -I$(STDPERPH_PATH)/inc -I$(NP2_ROOT)/src/Libraries/CMSIS/Include -I$(NP2_PATH)/inc -I. -I$(ENC_PATH) -I$(NET_PATH) -I$(NP2_PATH)/net -I$(NP2_ROOT)/src/net/apps/tuio
CFLAGS += -I$(ENC_PATH)/include #-I$(ENC_PATH)/include/arch -I$(ENC_PATH)/netif 
CFLAGS += -I$(NET_PATH)/src/include -I$(NET_PATH)/src/include/lwip -I$(NET_PATH)/src/include/ipv4 -I$(NET_PATH)/src/include/ipv6
CFLAGS += -I$(VCP_PATH)/inc -I$(USBCORE_PATH)/inc -I$(USBCDC_PATH)/inc -I$(USBOTG_PATH)/inc -I$(CLI_PATH)
CFLAGS += -I$(FREERTOS_PATH)/include -I$(FREERTOS_PATH)/portable/GCC/ARM_CM4F
CFLAGS += -I$(FATFS_PATH)
CFLAGS += -DENABLE_VCP #Enable USB VCP for debug_printf
CFLAGS += -DENABLE_FRVCP #Enable USB VCP for debug_printf with FreeRTOS
#CFLAGS += -DENABLE_DEBUG_UART	#Enable UART4 for debug printf
CFLAGS += -I$(TUIO_PATH)

SRCS += $(NP2_PATH)/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: proj

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -Llib -lm -lc -lgcc -lnosys #-lstm32f4
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

prog:
	sudo dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
