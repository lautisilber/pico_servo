add_library(PicoPioPWM)

pico_generate_pio_header(PicoPioPWM ${CMAKE_CURRENT_LIST_DIR}/pwm.pio)

target_sources(PicoPioPWM PRIVATE PicoPioPWM.c)

target_link_libraries(PicoPioPWM PUBLIC pico_stdlib hardware_pio hardware_clocks)

target_include_directories(PicoPioPWM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
