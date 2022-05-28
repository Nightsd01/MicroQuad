set(COMPONENT_ADD_INCLUDEDIRS "src")
file(GLOB COMPONENT_SRCS src/*.cc src/*.cpp src/*.c)

set(COMPONENT_REQUIRES
    arduino
)

set(COMPONENT_PRIV_REQUIRES)

register_component()
