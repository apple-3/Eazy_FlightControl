file(GLOB_RECURSE BSP_SOURCES "*.cpp" "*.c")
add_library(bsp_lib STATIC
    ${BSP_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f103
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f407

)
target_include_directories(bsp_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f407
)
target_compile_definitions(bsp_lib PUBLIC
    USE_HAL_DRIVER
    STM32F407xx
)
target_link_libraries(bsp_lib PUBLIC platform)