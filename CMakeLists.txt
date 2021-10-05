add_library(lvglDriver INTERFACE)

add_subdirectory(TARGET_STM32F407VE_BLACK)

target_include_directories(lvglDriver INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(lvglDriver
    INTERFACE
      LVGLDispDriverBase.cpp
      LVGLDispDriver_GC9A01.cpp
      LVGLDispDriver_ST7735.cpp
      LVGLInputDriverBase.cpp
      LVGLTouchDriverXPT2046.cpp
)

target_link_libraries(lvglDriver INTERFACE lvgldriver-stm32f407ve-black)