add_library(lvglDriver INTERFACE)

target_include_directories(lvglDriver INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(lvglDriver
    INTERFACE
      LVGLDispDriverBase.cpp
      LVGLDispDriver_GC9A01.cpp
      LVGLInputDriverBase.cpp
      LVGLTouchDriverXPT2046.cpp
)

