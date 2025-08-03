cmake_minimum_required(VERSION 3.12)

# Import the Pico SDK (make sure pico_sdk_import.cmake is in this directory)
include(pico_sdk_import.cmake)

project(dma_demo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# --------------------------------------------------------
# Sources for your project
# --------------------------------------------------------
add_executable(dma_demo
    dma_demo.cpp
    mcp4725.cpp
    audio_i2s.c        
	/Users/manisamet/school/pico_audio_i2s_32b/src/pico_audio_32b/audio.cpp
	/Users/manisamet/school/pico-extras/src/common/pico_util_buffer/buffer.c
	
)

# --------------------------------------------------------
# Generate header from the PIO program
# --------------------------------------------------------
pico_generate_pio_header(dma_demo ${CMAKE_CURRENT_LIST_DIR}/audio_i2s.pio)

# --------------------------------------------------------
# Enable stdio over USB (and disable UART)
# --------------------------------------------------------
pico_enable_stdio_usb(dma_demo 1)
pico_enable_stdio_uart(dma_demo 0)

# --------------------------------------------------------
# Link required libraries
# --------------------------------------------------------
target_link_libraries(
    dma_demo
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_i2c
    hardware_dma
    hardware_pio
    hardware_irq
    pico_multicore
)

# --------------------------------------------------------
# Include paths for headers
# --------------------------------------------------------
target_include_directories(dma_demo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}                                        # Your project root
    /Users/manisamet/school/pico_audio_i2s_32b/src/include           # for pico/audio_i2s.h
    /Users/manisamet/school/pico_audio_i2s_32b/src/pico_audio_32b/include  # for pico/audio.h
    /Users/manisamet/school/pico-extras/src/common/pico_util_buffer/include # for pico/util/buffer.h
)

# --------------------------------------------------------
# Generate additional output formats (UF2, bin, etc.)
# --------------------------------------------------------
pico_add_extra_outputs(dma_demo)
