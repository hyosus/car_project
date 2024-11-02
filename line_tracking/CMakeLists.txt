add_executable(car_project
        line_tracking.c
        )

target_link_libraries(car_project
        pico_stdlib              # for core functionality
        pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
        hardware_adc
        pico_multicore
        )

pico_enable_stdio_usb(car_project 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(car_project)

# add url via pico_set_program_url
example_auto_set_url(car_project)
