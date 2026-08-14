cmake_minimum_required(VERSION 3.16)

# Bump this when pushing an OTA - it's how you confirm over the network that the
# image you meant to ship is the one actually running, since there's no console.
set(PROJECT_VER "0.11.1")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_gateway)
