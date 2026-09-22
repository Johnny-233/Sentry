cmake_minimum_required(VERSION 3.22)

# 设置项目名称
set(CMAKE_PROJECT_NAME Sentry)

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
######################### 应用层: project/ 单入口 #########################
# project/ = 从 infantry_cpp 迁入的 C++ 框架, 现在是本工程唯一的固件应用。
#
# 旧 C 应用层 (application/ + Modules/ + bsp/) 仍留在仓库里, 但**不再**加进 CMake:
#   1) 与 project/ 大面积同名同功能 (daemon / remote / referee / ist8310 /
#      BMI088 / dji_motor / bsp_log / bsp_can / bsp_usart / bsp_dwt / crc ...)。
#      一起编译会重复定义符号 (例如 bsp_dwt.c 与 project 的 bsp_dwt.cpp 都定义
#      DWT_Init/DWT_GetTimeline_s), 而且 #include "daemon.h"、"crc.h" 会撞到
#      两套头文件——crc.h 已经因此把 project 那份改名成 crc_rm.h;
#   2) 入口不同: 旧层是 main.c 里的 RobotInit(); 新层是 Robot_Init()/Robot_Task()
#      (入口挂在 Src/freertos.c 的 MX_FREERTOS_Init)。两套调度不可能同时当固件。
# 需要旧实现时: 用 Reborn 分支 (它就是迁移前的 C 固件), 例如
#   git worktree add ../Sentry-legacy Reborn
#
# (下面这段是迁移前的老写法, 保留作对照, 不要取消注释)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/adc)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/can)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/dwt)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/flash)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/gpio)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/iic)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/log)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/pwm)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/spi)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/usart)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/usb)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/alarm)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/algorithm)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/BMI088)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/daemon)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/imu)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/ist8310)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/lqr)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/master_machine)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/matrix)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/message_center)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/DJImotor)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/DMmotor)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/MImotor)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/servo_motor)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/referee)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/remote)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/super_cap)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/Trajectory_planning)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/chassis)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/cmd)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/gimbal)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/gimbal_algorithm)
# USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/shoot)

######################### project/ : 全部用户代码只有这一套 #########################
# project/bsp/         外设抽象层 (对方的 C++ 类)
# project/modules/     可复用模块: 对方的模块 + 本仓自己补的 mi_motor / master_machine(seasky) / super_cap
# project/application/ 本仓整车控制逻辑的 C++ 移植版 (robot/chassis/gimbal/shoot/cmd/gimbal_algorithm/ui)
#
# 那些"迁移前的老 C 代码"(顶层 bsp/ + Modules/ + application/*.c)已经删掉, 只保留这一套要用的;
# 需要对照旧实现时从 git 取:  `git show HEAD:Modules/motor/DJImotor/dji_motor.c` 之类
# (删掉的顶层 121 个文件都在 git 里; 对方那套 project/application 在
#  D:\CubeMX Project\27_infantry\project\application 有逐字节相同的副本)。
# 注意 project/bsp/bsp_log/ 自带一份 SEGGER_RTT(.c/_Conf.h), 所以
# cmake/stm32cubemx/CMakeLists.txt 里已把 Middlewares 那份移出编译, 避免重复符号。
file(GLOB_RECURSE PROJECT_SRC CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/project/bsp/*.c
    ${CMAKE_SOURCE_DIR}/project/bsp/*.cpp
    ${CMAKE_SOURCE_DIR}/project/modules/*.c
    ${CMAKE_SOURCE_DIR}/project/modules/*.cpp
    ${CMAKE_SOURCE_DIR}/project/application/*.c
    ${CMAKE_SOURCE_DIR}/project/application/*.cpp
)
# 视觉/小电脑链路用本仓原本的 seasky 协议(project/modules/master_machine),
# 对方那套 20 字节帧的 minipc_comm 不编译(文件保留作参考)。
list(REMOVE_ITEM PROJECT_SRC ${CMAKE_SOURCE_DIR}/project/modules/minipc_comm/minipc_comm.cpp)
target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${PROJECT_SRC})

file(GLOB_RECURSE PROJECT_HDR CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/project/bsp/*.h
    ${CMAKE_SOURCE_DIR}/project/bsp/*.hpp
    ${CMAKE_SOURCE_DIR}/project/modules/*.h
    ${CMAKE_SOURCE_DIR}/project/modules/*.hpp
    ${CMAKE_SOURCE_DIR}/project/application/*.h
    ${CMAKE_SOURCE_DIR}/project/application/*.hpp
    ${CMAKE_SOURCE_DIR}/project/application/*/*.h
    ${CMAKE_SOURCE_DIR}/project/application/*/*.hpp
)
set(PROJECT_INC_DIRS "")
foreach(hdr ${PROJECT_HDR})
    get_filename_component(hdr_dir ${hdr} DIRECTORY)
    list(APPEND PROJECT_INC_DIRS ${hdr_dir})
endforeach()
list(REMOVE_DUPLICATES PROJECT_INC_DIRS)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${PROJECT_INC_DIRS})

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Building hex & bin file...\nEXCUTABLE SIZE:"
    COMMAND ${SIZE} ${PROJECT_NAME}.elf
)