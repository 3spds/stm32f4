# put your *.o targets here, make should handle the rest!

SRCS = main_filt_dsp.c system_stm32f4xx.c stm32f4xx_rcc.c stm32f4_discovery.c stm32f4xx_adc.c  stm32f4xx_dac.c  stm32f4xx_dma.c
SRCS += $(wildcard src/CMSIS/BasicMathFunctions/*.c)
SRCS += $(wildcard src/CMSIS/CommonTables/*.c)
SRCS += $(wildcard src/CMSIS/ComplexMathFunctions/*.c)
#SRCS += $(wildcard src/CMSIS/ControllerFunctions/*.c)
SRCS += $(wildcard src/CMSIS/FastMathFunctions/*.c)
SRCS += $(wildcard src/CMSIS/FilteringFunctions/*.c)
SRCS += $(wildcard src/CMSIS/MatrixFunctions/*.c)
#SRCS += $(wildcard src/CMSIS/StatisticsFunctions/*.c)
SRCS += $(wildcard src/CMSIS/SupportFunctions/*.c)
#SRCS += $(wildcard src/CMSIS/TransformFunctions/*.c)

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

# that's it, no need to change anything below this line!

###################################################

CC=/home/joe/Documents/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi-gcc
OBJCOPY=/home/joe/Documents/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi-objcopy

CFLAGS  = -g -O2 -Wall -T stm32_flash.ld -fno-strict-aliasing
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -I. -DARM_MATH_CM4 -D'__FPU_PRESENT=1' #this line might be bad...
CFLAGS += -mfloat-abi=hard
CFLAGS += -mfpu=fpv4-sp-d16
###################################################

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

CFLAGS += -Iinc -Ilib -Ilib/inc
CFLAGS += -Ilib/inc/core -Ilib/inc/peripherals

SRCS += lib/startup_stm32f4xx.s # add startup file to build



OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -Llib -lstm32f4 -lc -lm
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
