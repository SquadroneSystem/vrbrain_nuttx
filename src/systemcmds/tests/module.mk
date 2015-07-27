#
# Assorted tests and the like
#

MODULE_COMMAND		 = tests
MODULE_STACKSIZE	 = 12000
MAXOPTIMIZATION		 = -Os

ifneq ($(findstring PX4, $(CONFIG_BOARD)),)
SRCS			 = test_adc.c \
			   test_bson.c \
			   test_float.c \
			   test_gpio.c \
			   test_hott_telemetry.c \
			   test_hrt.c \
			   test_int.c \
			   test_jig_voltages.c \
			   test_led.c \
			   test_sensors.c \
			   test_servo.c \
			   test_sleep.c \
			   test_time.c \
			   test_uart_baudchange.c \
			   test_uart_console.c \
			   test_uart_loopback.c \
			   test_uart_send.c \
			   test_mixer.cpp \
			   test_mathlib.cpp \
			   test_file.c \
			   test_file2.c \
			   tests_main.c \
			   test_param.c \
			   test_ppm_loopback.c \
			   test_rc.c \
			   test_conv.cpp \
			   test_mount.c \
			   test_mtd.c
endif

ifneq ($(findstring VRBRAIN_V4, $(CONFIG_BOARD)),)
SRCS			 = test_adc.c \
			   test_hrt.c \
			   test_led.c \
			   test_sensors.c \
			   tests_main.c \
			   test_rc.c \
			   test_mtd.c \
			   test_buzzer.c \
			   test_uart_bridge.c
endif

ifneq ($(findstring VRBRAIN_V5, $(CONFIG_BOARD)),)
SRCS			 = test_adc.c \
			   test_hrt.c \
			   test_led.c \
			   test_sensors.c \
			   tests_main.c \
			   test_rc.c \
			   test_mtd.c \
			   test_buzzer.c \
			   test_uart_bridge.c
endif

ifneq ($(findstring VRUBRAIN_V5, $(CONFIG_BOARD)),)
SRCS			 = test_adc.c \
			   test_hrt.c \
			   test_led.c \
			   test_sensors.c \
			   tests_main.c \
			   test_rc.c \
			   test_mtd.c \
			   test_buzzer.c \
			   test_uart_bridge.c
endif

ifneq ($(findstring HBRAIN_V0, $(CONFIG_BOARD)),)
SRCS			 = test_adc.c \
			   test_hrt.c \
			   test_led.c \
			   test_sensors.c \
			   tests_main.c \
			   test_rc.c \
			   test_mtd.c \
			   test_buzzer.c \
			   test_uart_bridge.c
endif

ifneq ($(findstring HBRAIN_V1, $(CONFIG_BOARD)),)
SRCS			 = test_adc.c \
			   test_hrt.c \
			   test_led.c \
			   test_sensors.c \
			   tests_main.c \
			   test_rc.c \
			   test_mtd.c \
			   test_buzzer.c \
			   test_uart_bridge.c
endif
			   
ifneq ($(findstring HBRAIN_V2, $(CONFIG_BOARD)),)
SRCS			 = test_adc.c \
			   test_hrt.c \
			   test_led.c \
			   test_sensors.c \
			   tests_main.c \
			   test_rc.c \
			   test_mtd.c \
			   test_buzzer.c \
			   test_uart_bridge.c
endif
