# Need to rewrite this file!

COMP_FLAGS = arm-none-eabi-gcc $(COMP_INCLUDES) -Wall -mthumb -mcpu=cortex-m0 -g -S
ASS_FLAGS = arm-none-eabi-as -mthumb -mcpu=cortex-m0 -g
LINK_FLAGS = arm-none-eabi-gcc -mthumb -mcpu=cortex-m0 -nostartfiles -T stm32f051_linker_script.ld

COMP_INCLUDES = -I ./Libraries/

# LINKING

main.elf: main.o stm32f051_startup.o eeprom_lib.o woodboxstm32f051C6.o temp_sensor_lib.o lcd_stm32f0.o
	$(LINK_FLAGS) -o main.elf main.o stm32f051_startup.o eeprom_lib.o woodboxstm32f051C6.o temp_sensor_lib.o lcd_stm32f0.o

# ASSEMBLING

stm32f051_startup.o: stm32f051_startup.s
	$(ASS_FLAGS) -o stm32f051_startup.o stm32f051_startup.s

eeprom_lib.o: eeprom_lib.s
	$(ASS_FLAGS) -o eeprom_lib.o eeprom_lib.s

temp_sensor_lib.o: temp_sensor_lib.s
	$(ASS_FLAGS) -o temp_sensor_lib.o temp_sensor_lib.s

woodboxstm32f051C6.o: woodboxstm32f051C6.s
	$(ASS_FLAGS) -o woodboxstm32f051C6.o woodboxstm32f051C6.s

lcd_stm32f0.o: lcd_stm32f0.s
	$(ASS_FLAGS) -o lcd_stm32f0.o lcd_stm32f0.s

main.o: main.s
	$(ASS_FLAGS) -o main.o main.s

# COMPILING

eeprom_lib.s: eeprom_lib.c
	$(COMP_FLAGS) -o eeprom_lib.s eeprom_lib.c

woodboxstm32f051C6.s: woodboxstm32f051C6.c
	$(COMP_FLAGS) -o woodboxstm32f051C6.s woodboxstm32f051C6.c

temp_sensor_lib.s: temp_sensor_lib.c
	$(COMP_FLAGS) -o temp_sensor_lib.s temp_sensor_lib.c

lcd_stm32f0.s: lcd_stm32f0.c
	$(COMP_FLAGS) -o lcd_stm32f0.s lcd_stm32f0.c

main.s: main.c
	$(COMP_FLAGS) -o main.s main.c
