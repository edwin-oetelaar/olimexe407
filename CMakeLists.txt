cmake_minimum_required(VERSION 3.2)
project(olimexe407 C)
set(VERBOSE ON)
# olimex STM32F407ZGT6 use define STM32F40_41xxx
SET(CMAKE_SYSTEM_NAME Generic) # target is embedded with no OS

SET(CMAKE_C_COMPILER arm-none-eabi-gcc) # choose toolchain, will also setup linker binutils etc
# SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)

SET(CSTANDARD "-std=gnu99")
SET(CDEBUG "-g3")
SET(CWARN "-Wall -Wstrict-prototypes")

SET(COPT "-Os")

SET(CMCU "-mcpu=cortex-m4")
SET(CDEFS "-specs=nano.specs -fgcse -fexpensive-optimizations -fcse-skip-blocks -fcse-follow-jumps -fdata-sections -ffunction-sections -ffunction-sections -fsingle-precision-constant -mthumb -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wl,--verbose,--no-undefined,-T,${PROJECT_SOURCE_DIR}/stm32f407zg_flash.ld ")


SET(CFLAGS "${CMCU} ${CDEBUG} ${CDEFS} ${CINCS} ${COPT} ${CWARN} ${CSTANDARD} ${CEXTRA}")
SET(CXXFLAGS "${CMCU} ${CDEFS} ${CINCS} ${COPT}")

SET(CMAKE_C_FLAGS ${CFLAGS})
SET(CMAKE_CXX_FLAGS ${CXXFLAGS})


# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    cmsis/arm_common_tables.h
    cmsis/arm_math.h
    cmsis/core_cm4.h
    cmsis/core_cmSimd.h
    cmsis/core_cmFunc.h
    cmsis/core_cmInstr.h
    FreeRTOS/inc/croutine.h
    FreeRTOS/inc/event_groups.h
    FreeRTOS/inc/FreeRTOS.h
    FreeRTOS/inc/list.h
    FreeRTOS/inc/mpu_wrappers.h
    FreeRTOS/inc/portable.h
    FreeRTOS/inc/portmacro.h
    FreeRTOS/inc/projdefs.h
    FreeRTOS/inc/queue.h
    FreeRTOS/inc/semphr.h
    FreeRTOS/inc/StackMacros.h
    FreeRTOS/inc/task.h
    FreeRTOS/inc/timers.h
    FreeRTOS/src/croutine.c
    FreeRTOS/src/event_groups.c
    FreeRTOS/src/heap_4.c
    FreeRTOS/src/list.c
    FreeRTOS/src/port.c
    FreeRTOS/src/queue.c
    FreeRTOS/src/tasks.c
    FreeRTOS/src/timers.c
    inc/FreeRTOSConfig.h
    inc/my_queue.h
    inc/stm32f4xx.h
    inc/stm32f4xx_conf.h
    inc/system_stm32f4xx.h
    inc/utils_edwin.h
    SPL/inc/misc.h
    SPL/inc/stm32f4xx_adc.h
    SPL/inc/stm32f4xx_can.h
    SPL/inc/stm32f4xx_crc.h
    SPL/inc/stm32f4xx_cryp.h
    SPL/inc/stm32f4xx_dac.h
    SPL/inc/stm32f4xx_dbgmcu.h
    SPL/inc/stm32f4xx_dcmi.h
    SPL/inc/stm32f4xx_dma.h
    SPL/inc/stm32f4xx_dma2d.h
    SPL/inc/stm32f4xx_exti.h
    SPL/inc/stm32f4xx_flash.h
    SPL/inc/stm32f4xx_flash_ramfunc.h
 #   SPL/inc/stm32f4xx_fmc.h
 #   SPL/inc/stm32f4xx_fsmc.h
    SPL/inc/stm32f4xx_gpio.h
    SPL/inc/stm32f4xx_hash.h
    SPL/inc/stm32f4xx_i2c.h
    SPL/inc/stm32f4xx_iwdg.h
    SPL/inc/stm32f4xx_ltdc.h
    SPL/inc/stm32f4xx_pwr.h
    SPL/inc/stm32f4xx_rcc.h
    SPL/inc/stm32f4xx_rng.h
    SPL/inc/stm32f4xx_rtc.h
    SPL/inc/stm32f4xx_sai.h
    SPL/inc/stm32f4xx_sdio.h
    SPL/inc/stm32f4xx_spi.h
    SPL/inc/stm32f4xx_syscfg.h
    SPL/inc/stm32f4xx_tim.h
    SPL/inc/stm32f4xx_usart.h
    SPL/inc/stm32f4xx_wwdg.h
    SPL/src/misc.c
    SPL/src/stm32f4xx_adc.c
    SPL/src/stm32f4xx_can.c
    SPL/src/stm32f4xx_crc.c
    SPL/src/stm32f4xx_cryp.c
    SPL/src/stm32f4xx_cryp_aes.c
    SPL/src/stm32f4xx_cryp_des.c
    SPL/src/stm32f4xx_cryp_tdes.c
    SPL/src/stm32f4xx_dac.c
    SPL/src/stm32f4xx_dbgmcu.c
    SPL/src/stm32f4xx_dcmi.c
    SPL/src/stm32f4xx_dma.c
    SPL/src/stm32f4xx_dma2d.c
    SPL/src/stm32f4xx_exti.c
    SPL/src/stm32f4xx_flash.c
    SPL/src/stm32f4xx_flash_ramfunc.c
  #  SPL/src/stm32f4xx_fmc.c
  #  SPL/src/stm32f4xx_fsmc.c
    SPL/src/stm32f4xx_gpio.c
    SPL/src/stm32f4xx_hash.c
    SPL/src/stm32f4xx_hash_md5.c
    SPL/src/stm32f4xx_hash_sha1.c
    SPL/src/stm32f4xx_i2c.c
    SPL/src/stm32f4xx_iwdg.c
    SPL/src/stm32f4xx_ltdc.c
    SPL/src/stm32f4xx_pwr.c
    SPL/src/stm32f4xx_rcc.c
    SPL/src/stm32f4xx_rng.c
    SPL/src/stm32f4xx_rtc.c
    SPL/src/stm32f4xx_sai.c
    SPL/src/stm32f4xx_sdio.c
    SPL/src/stm32f4xx_spi.c
    SPL/src/stm32f4xx_syscfg.c
    SPL/src/stm32f4xx_tim.c
    SPL/src/stm32f4xx_usart.c
    SPL/src/stm32f4xx_wwdg.c
    src/main.c
    src/system_stm32f4xx.c
    src/utils_edwin.c
    src/syscalls.h
    src/syscalls.c
    )

include_directories(FreeRTOS/inc)
include_directories(inc)
include_directories(SPL/inc)
include_directories(cmsis)
include_directories(cmsis/Lib)
# SET(CMAKE_EXE_LINKER_FLAGS "-static")
# MESSAGE(STATUS "link flags" ${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} )
SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")

add_definitions(-DARM_MATH_CM4=1)
add_definitions(-D__FPU_USED=1)
add_definitions(-DUSE_STDPERIPH_DRIVER=1)
add_definitions(-D__ASSEMBLY__)
add_definitions(-DSTM32F40_41xxx)
add_definitions(-DSTM32F40_41xxx)
add_definitions(-DHSE_VALUE=12000000)
add_definitions(-D__FPU_PRESENT=1)

SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
SET(BUILD_SHARED_LIBRARIES OFF)



add_executable(olimexe407 ${SOURCE_FILES} )

target_link_libraries(olimexe407 c m )
link_directories(${PROJECT_SOURCE_DIR}/cmsis/Lib/GCC)
target_link_libraries(olimexe407 ${PROJECT_SOURCE_DIR}/cmsis/Lib/GCC/libarm_cortexM4lf_math.a )
# bash# arm-none-eabi-gcc -I. -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb main.c
#bash# arm-none-eabi-ld -Tstm32.ld -nostartfiles -o main.elf main.o
# bash# arm-none-eabi-objcopy -Obinary main.elf main.bin