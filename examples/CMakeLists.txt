add_executable(sweep_example)

target_sources(sweep_example PRIVATE sweep.c)

target_link_libraries(sweep_example PRIVATE pico_stdlib PicoPioPWM)

pico_enable_stdio_usb(sweep_example 1)
pico_enable_stdio_uart(sweep_example 1)


pico_add_extra_outputs(sweep_example)