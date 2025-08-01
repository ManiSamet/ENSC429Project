cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(dma_demo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROEJECT_SOURCE_DIR})

pico_sdk_init()

add_executable(dma_demo 
			dma_demo.cpp
			mcp4725.cpp
			)

pico_enable_stdio_usb(dma_demo 1)
pico_enable_stdio_uart(dma_demo 0)

target_link_libraries(
	dma_demo 
	pico_stdlib
	hardware_pwm
	hardware_adc
	hardware_i2c
    hardware_dma
	
)
target_include_directories(dma_demo PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

pico_add_extra_outputs(dma_demo)

## 2. Build directory
#mkdir build
#cd build

# 3. (MacOS) To run CMake and fetch the Pico SDK if you dont want to clone on to your machine
#cmake .. -DPICO_SDK_FETCH_FROM_GIT=1

# 4. (Windows) To run CMake and fetch the Pico SDK 
#...

#5. /build to the .gitignore file