add_library(lvgldriver-stm32f407ve-black INTERFACE)

target_include_directories(lvgldriver-stm32f407ve-black INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(lvgldriver-stm32f407ve-black
    INTERFACE
      ili9341_fsmc.cpp
      LVGLDispDriverSTM32F407VE_BLACK.cpp
      LVGLTouchDriver_STM32F407VE_BLACK.cpp
)

