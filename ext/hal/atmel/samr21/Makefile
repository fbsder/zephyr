
ifdef CONFIG_HAS_ATMEL_SAMR21_HAL
#ccflags-y += -I$(srctree)/ext/hal/atmel/samr21/include -I$(srctree)/ext/hal/cmsis/include
#ccflags-y += -D__ATSAMR21G18A__ -DDONT_USE_CMSIS_INIT

ZEPHYRINCLUDE += -I$(srctree)/ext/hal/atmel/samr21/include -I$(srctree)/ext/hal/cmsis/include
CFLAGS += -D__ATSAMR21G18A__ -DDONT_USE_CMSIS_INIT
endif
