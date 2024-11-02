add_executable(barcode
        barcode.c
        )

target_link_libraries(barcode
        pico_stdlib              # for core functionality
        pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
        hardware_adc
        hardware_gpio
        )

pico_enable_stdio_usb(barcode 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(barcode)

# add url via pico_set_program_url
example_auto_set_url(barcode)
