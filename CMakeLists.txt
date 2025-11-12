# -- CMAKE for Espressif IDF
# -- author MLX
# -- copyright GPLv3

idf_component_register(
    SRC_DIRS src
    INCLUDE_DIRS src
    REQUIRES esp_common freertos hal log nvs_flash driver esp_timer
)

target_compile_options(${COMPONENT_LIB} PUBLIC -DESP32_CMAKE=1 -Wno-error -Wno-format -fpermissive)
