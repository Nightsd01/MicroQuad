file(GLOB SOURCES src/*.cc src/*.cpp src/*.c)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES arduino MicroQuad
)

# Suppress string truncation warning for this third-party library
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -Wno-stringop-truncation
    -Wno-implicit-fallthrough
)
