add_executable(line_tracking
        line_tracking.c
        )

target_link_libraries(line_tracking
        pico_stdlib              # for core functionality
        pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
        hardware_adc
        pico_multicore
        )

pico_enable_stdio_usb(line_tracking 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(line_tracking)

# add url via pico_set_program_url
example_auto_set_url(line_tracking)
