add_library(PicoServo STATIC
    PicoServo.c
)

add_subdirectory(lib/PicoPioPWM)

target_include_directories(PicoServo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(PicoServo PRIVATE
    pico_stdlib
    pico_sync
)

if (NOT TARGET PicoPioPWM)
    message(FATAL_ERROR "Didn't load PicoPioPWM")
endif()

target_link_libraries(PicoServo PUBLIC
    PicoPioPWM
)