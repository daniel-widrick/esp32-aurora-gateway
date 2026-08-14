idf_component_register(
    SRCS "main.c" "aurora_modbus.c" "poller.c" "wifi.c" "api.c" "logbuf.c" "ota.c"
         "supervisor.c" "auth.c" "settings.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_wifi esp_netif esp_event nvs_flash
             esp_https_server json esp_app_format esp_timer app_update esp_system
    EMBED_TXTFILES "certs/servercert.pem" "certs/prvtkey.pem" "www/index.html"
)
