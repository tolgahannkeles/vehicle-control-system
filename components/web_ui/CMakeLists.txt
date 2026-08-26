idf_component_register(
    SRCS "web_ui.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server motor_driver ota_manager log esp_timer
    EMBED_FILES "index.html"
)