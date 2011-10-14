ifeq ($(CONFIG_ARCH_MSM7201A),y)
  zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x10800000
endif

ifeq ($(CONFIG_ARCH_MSM7225),y)
  zreladdr-y        := 0x00208000
params_phys-y       := 0x00200100
initrd_phys-y       := 0x00A00000
endif

ifeq ($(CONFIG_ARCH_MSM7625),y)
  zreladdr-y		:= 0x1B408000
params_phys-y		:= 0x1B400100
initrd_phys-y		:= 0x1BC00000
endif

