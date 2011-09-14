obj-y += io.o idle.o irq.o timer.o dma.o memory.o
obj-y += devices.o
obj-y += devices_htc.o
obj-y += proc_comm.o
obj-y += vreg.o
obj-y += acpuclock.o
obj-y += clock.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += drv_callback.o
obj-y += htc_port_list.o
obj-y += msm_vibrator.o

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
# Schlund: temporal fix to disable adsp to avoid issues
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_MSM_CPU_FREQ) += cpufreq.o

obj-y += htc_wifi_nvs.o htc_wifi.o
obj-$(CONFIG_TOUCHSCREEN_TSSC_MANAGER) += htc_tp_cal.o
obj-$(CONFIG_HTC_ACOUSTIC) += htc_acoustic.o
obj-$(CONFIG_SENSORS_AKM8976) += htc_akm_cal.o

obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o

# MSM7225 boards
obj-$(CONFIG_MACH_BAHAMAS) += board-bahamas.o board-click-panel.o board-bahamas-keypad.o
obj-$(CONFIG_MACH_BAHAMAS) += board-bahamas-mmc.o
obj-$(CONFIG_MACH_BAHAMAS) += board-bahamas-rfkill.o

obj-$(CONFIG_MACH_MEGA) += dmesg_log.o
obj-$(CONFIG_MACH_MEGA) += board-mega.o board-mega-panel.o board-mega-keypad.o
obj-$(CONFIG_MACH_MEGA) += board-mega-mmc.o
obj-$(CONFIG_MACH_MEGA) += board-mega-rfkill.o
obj-$(CONFIG_MACH_MEGA) += htc_fb_console.o fonts.o font_pearl_8x8.o

obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
obj-$(CONFIG_HTC_AUDIOJACK) += htc_audio_jack.o
