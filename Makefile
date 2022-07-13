# PKG_CONFIG_PATH=C:\GTK\lib\pkgconfig
# CC   = mingw32-gcc
# CC = /usr/gcc/4.3/bin/gcc -m64
# набери:
# $ arm-none-eabi-gcc -print-multi-lib
# $ arm-none-eabi-gcc --target-help
# $ arm-eabi-gcc --target-help
# чтобы понять какие опции можно сочетать
# mthumb@march=armv7e-m@mfloat-abi=hard@mfpu=fpv4-sp-d16
# arm/v5te/softfp;@marm@march=armv5te+fp@mfloat-abi=softfp
# arm/v5te/hard;@marm@march=armv5te+fp@mfloat-abi=hard
# thumb/nofp;@mthumb@mfloat-abi=soft
# thumb/v7/nofp;@mthumb@march=armv7@mfloat-abi=soft
# thumb/v7+fp/softfp;@mthumb@march=armv7+fp@mfloat-abi=softfp
# thumb/v7+fp/hard;@mthumb@march=armv7+fp@mfloat-abi=hard
# thumb/v7-r+fp.sp/softfp;@mthumb@march=armv7-r+fp.sp@mfloat-abi=softfp
# thumb/v7-r+fp.sp/hard;@mthumb@march=armv7-r+fp.sp@mfloat-abi=hard
# thumb/v6-m/nofp;@mthumb@march=armv6s-m@mfloat-abi=soft
# thumb/v7-m/nofp;@mthumb@march=armv7-m@mfloat-abi=soft
# thumb/v7e-m/nofp;@mthumb@march=armv7e-m@mfloat-abi=soft
# thumb/v7e-m+fp/softfp;@mthumb@march=armv7e-m+fp@mfloat-abi=softfp
# thumb/v7e-m+fp/hard;@mthumb@march=armv7e-m+fp@mfloat-abi=hard
# thumb/v7e-m+dp/softfp;@mthumb@march=armv7e-m+fp.dp@mfloat-abi=softfp
# thumb/v7e-m+dp/hard;@mthumb@march=armv7e-m+fp.dp@mfloat-abi=hard
# thumb/v8-m.base/nofp;@mthumb@march=armv8-m.base@mfloat-abi=soft
# thumb/v8-m.main/nofp;@mthumb@march=armv8-m.main@mfloat-abi=soft
# thumb/v8-m.main+fp/softfp;@mthumb@march=armv8-m.main+fp@mfloat-abi=softfp
# thumb/v8-m.main+fp/hard;@mthumb@march=armv8-m.main+fp@mfloat-abi=hard
# thumb/v8-m.main+dp/softfp;@mthumb@march=armv8-m.main+fp.dp@mfloat-abi=softfp
# thumb/v8-m.main+dp/hard;@mthumb@march=armv8-m.main+fp.dp@mfloat-abi=hard
# thumb/v8.1-m.main+mve/hard;@mthumb@march=armv8.1-m.main+mve@mfloat-abi=hard
# набери:
# $ arm-eabi-gcc -print-multi-lib
# thumb/v7-a+fp/hard;@mthumb@march=armv7-a+fp@mfloat-abi=hard
# thumb/v7-a+simd/hard;@mthumb@march=armv7-a+simd@mfloat-abi=hard
# thumb/v7ve+simd/hard;@mthumb@march=armv7ve+simd@mfloat-abi=hard
# thumb/v8-a+simd/hard;@mthumb@march=armv8-a+simd@mfloat-abi=hard
# thumb/v7e-m+fp/hard;@mthumb@march=armv7e-m+fp@mfloat-abi=hard
# thumb/v8-m.base/nofp;@mthumb@march=armv8-m.base@mfloat-abi=soft
# thumb/v8-m.main+fp/hard;@mthumb@march=armv8-m.main+fp@mfloat-abi=hard
# $ riscv64-unknown-elf-gcc -print-multi-lib
# \see https://en.wikipedia.org/wiki/RISC-V#ISA_base_and_extensions
# rv32iac/ilp32;@march=rv32iac@mabi=ilp32 -- без умножения
# rv32imac/ilp32;@march=rv32imac@mabi=ilp32 -- это подходит
# rv32imafc/ilp32f;@march=rv32imafc@mabi=ilp32f -- с FPU


CLANG ?= clang -target arm-none-eabi
#-I/c/Program\ Files\ \(x86\)/GNU\ Tools\ ARM\ Embedded/9\ 2019-q4-major/arm-none-eabi/include 

# список переменных в нижнем регистре
CROSS ?= arm-none-eabi-
DEVICE?= dispenser
CHIP  ?= STM32F4XX
CHIPX ?= STM32F407xx
ARCH  ?= CORTEXM4

include device/$(DEVICE).mak
# переменная CHIPX_ -- модель микроконтроллера в нижнем регистре
CHIPX_ = $(shell echo $(CHIPX) | tr A-Z a-z)
ifeq ($(ARCH), ARM926)
    OPTIMIZATION += -march=armv5te
else ifeq ($(ARCH), CORTEXM0)
    OPTIMIZATION += -mthumb -mcpu=cortex-m0
else ifeq ($(ARCH), CORTEXM3)
    OPTIMIZATION += -mthumb -mcpu=cortex-m3
else ifeq ($(ARCH), CORTEXM4)
    OPTIMIZATION += -mthumb -mcpu=cortex-m4
else ifeq ($(ARCH), CORTEXM4F)
    OPTIMIZATION += -mthumb -mcpu=cortex-m4 -march=armv7e-m+fp -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c11 -ffast-math
else ifeq ($(ARCH), CORTEXM7F)
# v7E-M + FPv5-SP-D16-M	-- Processor with single-precision floating-point
# v7E-M + FPv5-DP-D16-M	-- Processor with single-precision and double-precision floating-point
    OPTIMIZATION += -mthumb -mcpu=cortex-m7 -march=armv7e-m+fpv5 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -std=c11 -ffast-math -flto
else ifeq ($(ARCH), CORTEXM23)
#
    OPTIMIZATION += -mthumb -mcpu=cortex-m23 -std=c11 -mfloat-abi=soft
else ifeq ($(ARCH), CORTEXM33)
# ARMv8-M + FPv5-SP-D16-M	-- single-precision floating-point arithmetic operations only
# ARMv8-M + FPv5-D16-M	    -- double-precision arithmetic operations
#  -march=armv8-m.main+fp  -flto
    OPTIMIZATION += -mthumb -mcpu=cortex-m33 -march=armv8-m.main+dsp+fp -mfloat-abi=hard -mfpu=fpv5-sp-d16 -std=c11 -ffast-math
else ifeq ($(ARCH), CORTEXM55)
# Armv8.1-M architecture
# FPv5-SP-D16-M -- Single-precision arithmetic operations only
# FPv5-D16-M 	-- Single and double-precision arithmetic operations
    OPTIMIZATION += -mthumb -mcpu=cortex-m55 -march=armv8.1-m.main+mve.fp -mfloat-abi=hard -mfpu=fpv5-sp-d16 -std=c11 -ffast-math
else ifeq ($(ARCH), CORTEXR5F)
# 	VFPv3-D16 -- ARM Vector Floating Point v3 architecture, with 16 double-precision registers
    OPTIMIZATION += -mthumb -mcpu=cortex-r5 -march=armv7-r+vfpv3-d16 -mfloat-abi=hard -mfpu=vfpv3-d16 -std=c11
else ifeq ($(ARCH), RV32IMAC)
# integer mul/div atomic compressed 
    OPTIMIZATION += -march=rv32imac -mabi=ilp32  -std=c11
else ifeq ($(ARCH), RV32IMAFC)
# integer mul/div atomic compressed FPU
    OPTIMIZATION += -march=rv32imafc -mabi=ilp32f  -std=c11
else
    OPTIMIZATION += -march=armv4t
endif

CC = $(CROSS)gcc $(OPTIMIZATION)
#CC = $(CLANG) $(OPTIMIZATION)
AS = $(CROSS)as $(OPTIMIZATION)
AR = $(CROSS)ar
RANLIB = $(CROSS)ranlib

CROSSLIB = lib/$(ARCH)/

SIZE = $(CROSS)size
OBJCOPY = $(CROSS)objcopy
#  -fvisibility=default|hidden
CFLAGS +=  -Werror -D$(CHIP) -D$(CHIPX) -Wa,-aglhmns="./lst/$(@F).lst"
# -g3  -Wall
CFLAGS += -Os \
	-I./ -I./device/$(DEVICE) \
	-I./hal/CMSIS/include \
	-I./hal/$(CHIP) \
	-I./r3core 
# -I/mingw64/arm-none-eabi/include
# -lnosys  -specs=nano.specs -s -fPIC -lc -lm
LDFLAGS = -nostartfiles -nostdlib -nodefaultlibs  -Wl,-T -Xlinker "device/$(DEVICE)/$(CHIPX_).ld" -Wl,-Map=$(OUTPUT).map,--cref -fshort-wchar
#-lsocket -lusb
# первым делом прикомпиливается таблица векторов прерываний и код инициализации микроконтроллера
ASMSRC = \
	device/$(DEVICE)/startup_$(CHIPX_).s
SRC += \
	device/$(DEVICE)/low_level_init.c

#	hal/usart_drv.c
#	hal/$(CHIP)/usart.c \
#	hal/usart_drv.c \
# отдельно выделяем уровень абстракции аппаратуры HAL
SRC += \
	hal/$(CHIP)/pio.c \
	hal/$(CHIP)/adc.c \
	hal/$(CHIP)/pvd.c \
	hal/$(CHIP)/iwdg.c \
	hal/$(CHIP)/dma.c \
	hal/$(CHIP)/iflash.c
# временно
#	hal/$(CHIP)/crc.c

ifeq ($(ARCH), CORTEXM4F)
SRC += \
	hal/$(CHIP)/rtc.c \
	hal/$(CHIP)/mpu.c
endif

# драйвер USB FS и драйвер класса устройства CDC (Virtual Com Port)
# из usbd_cdc убрали динамические данные _malloc и _free
#	lib/usb/usbd_cdc_if.c
#	lib/usb/usb_device.c

ifdef USB_DEVICE
SRC += \
	hal/$(CHIP)/stm32f3xx_hal_pcd.c \
	hal/$(CHIP)/stm32f3xx_hal_pcd_ex.c \
	hal/$(CHIP)/stm32f3xx_ll_usb.c \
	lib/usb/usbd_core.c \
	lib/usb/usbd_ctlreq.c \
	lib/usb/usbd_ioreq.c \
	lib/usb/usbd_desc.c \
	lib/usb/usbd_conf.c \
	lib/usb/usbd_cdc.c

endif
# Операционная система
#	r3core/mem_block.c
#	r3core/r3_slice.c
#	r3core/r3rtc.c
#	r3core/timer.c
#	r3core/rtc.c 

#SRC +=	r3core/magma.c
#	r3core/aes.c 
#	r3core/stribog.c 
#	r3core/mail.c 
#	r3core/message.c 
#	r3core/mutex.c 
#	r3core/pipe.c 
#	r3core/queue.c 
#	r3core/semaphore.c 
#	r3core/async_queue.c 
#	r3core/r3_slist.c 

# Библиотека libc
SRC += \
	r3core/r3_slice.c \
	r3core/r3_stdlib_memcpy.c \
	r3core/r3_stdlib_memset.c \
	r3core/r3_stdlib.c
#	r3core/r3_stdlib_strchr.c 
#	r3core/r3_stdlib_strcmp.c 
#	r3core/r3_stdlib_strcpy.c 
#	r3core/r3_stdlib_strncmp.c 
#	r3core/r3_stdlib_strncpy.c 
#	r3core/r3_stdlib_strncat.c
#	r3core/r3_stdlib_memmove.c 
#	r3core/r3_stdlib_memcmp.c 
#	r3core/r3_stdlib_utf8.c 
#	r3core/c11_mutex.c
#	r3core/c11_cond.c
#	r3core/bsearch.c 

#	r3core/r3_object.c 
#	r3core/r3_asn.c
#	r3core/c11_tss.c 

SRC += \
	r3core/kernel.c \
	r3core/tracer.c \
	r3core/thread.c \
	r3core/svc.c  \
	r3core/timer.c \
	r3core/malloc.c \
	r3core/main.c \
	r3core/service.c

ifdef USE_R3CODE
SRC += \
	r3core/crc16.c \
	r3core/config.c
endif




ifdef BACNET_SERVER
SRC += \
	bacnet/bacnet_xml.c \
	bacnet/bacnet_apdu.c \
	bacnet/bacnet_bvlc.c \
	bacnet/bacnet_bvll.c \
	bacnet/bacnet_datalink.c \
	bacnet/bacnet_device.c \
	bacnet/bacnet_encode.c \
	bacnet/bacnet_file.c \
	bacnet/bacnet_mstp.c \
	bacnet/bacnet_object.c \
	bacnet/bacnet_router.c \
	bacnet/bacnet_terminal.c
endif
	# прикладное ПО ...
# SRC += tim_acusto.c
# SRC+= simon.c tim_pwm.c
#SRC+=r3core/arp.c
#	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
#	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
#	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \

OUTPUT=bin/$(DEVICE)-$(CHIP)-firmware
INSTALL=smarthouse
ASM_OBJECTS := $(ASMSRC:.s=.o)

all: $(OUTPUT)

$(OUTPUT): $(ASM_OBJECTS) $(SRC:.c=.o)
	$(CC) -dy -o $(OUTPUT).elf $^ -Wl,--no-whole-archive $(LDFLAGS) -n -L$(CROSSLIB)
	$(OBJCOPY) --strip-debug --strip-unneeded $(OUTPUT).elf  -O binary $(OUTPUT).bin
	$(SIZE) $^ $(OUTPUT).elf


$(ASM_OBJECTS): %.o : %.s
	$(CC) $(ASFLAGS) -c -o $@ $<

clean:
	rm -f $(OUTPUT) $(SRC:.c=.o) $(CMSYS_SRC:.c=.o) $(LIBC_SRC:.c=.o) $(OPENVG_SRC:.c=.o) $(ASM_OBJECTS)
