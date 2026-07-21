cmake_minimum_required(VERSION 3.22)

# 设置项目名称
set(CMAKE_PROJECT_NAME Infantry)

# 设置 C 标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

#####################添加C++编译与版本配置#################
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(SIZE arm-none-eabi-size) 

# 定义构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 启用编译命令导出（便于 clangd 等工具索引）
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Add color diagnostics
set(CMAKE_COLOR_DIAGNOSTICS ON)

# 包含工具链文件（必须在 project() 前设置编译器）
include("cmake/gcc-arm-none-eabi.cmake")

# 核心项目设置
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})



# 启用 C 和 ASM 语言支持
enable_language(C CXX ASM)

# Remove warning
add_compile_options(-w)

# 创建可执行文件
add_executable(${CMAKE_PROJECT_NAME})

target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)
# 添加 STM32CubeMX 生成的源码
add_subdirectory(cmake/stm32cubemx)

##############以下为修改过的cmakelist#################
function(USER_ADD_DIR dir)
    file(GLOB USER_SOURCE
        ${dir}/*.h
        ${dir}/*.c
        ${dir}/*.hpp
        ${dir}/*.cpp
        ${dir}/*.cc
    )
    target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${USER_SOURCE})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${dir})
endfunction()
###############################在此处添加你的文件路径#########
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/adc)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/can)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/dwt)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/flash)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/gpio)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/iic)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/log)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/pwm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/spi)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/usart)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/usb)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/alarm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/algorithm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/BMI088)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/daemon)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/imu)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/ist8310)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/lqr)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/master_machine)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/matrix)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/message_center)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/DJImotor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/DMmotor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/MImotor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/servo_motor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/referee)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/remote)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/super_cap)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/Trajectory_planning)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/chassis)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/cmd)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/gimbal)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/gimbal_algorithm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/shoot)

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Building hex & bin file...\nEXCUTABLE SIZE:"
    COMMAND ${SIZE} ${PROJECT_NAME}.elf
)