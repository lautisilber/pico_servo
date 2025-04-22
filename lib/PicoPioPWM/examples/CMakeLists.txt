add_executable(sweep_example)

target_sources(sweep_example PRIVATE sweep.c)

target_link_libraries(sweep_example PRIVATE pico_stdlib PicoPioPWM)
pico_add_extra_outputs(sweep_example)