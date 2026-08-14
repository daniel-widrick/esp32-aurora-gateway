#pragma once

#include "esp_err.h"

/* Start the HTTPS server (TLS, embedded self-signed cert). */
esp_err_t api_start(void);
