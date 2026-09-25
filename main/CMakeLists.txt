idf_component_register(
    SRCS "main.c" "si4432.c" "display.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_spi esp_driver_gpio esp_driver_rmt esp_driver_ledc esp_lcd esp_timer
)
