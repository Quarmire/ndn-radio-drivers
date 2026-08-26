# ESP32-C5 named-data radio — BLE bearer (extended advertising + scanning over the ND serial protocol).
# Build: source ~/esp/esp-idf/export.sh ; idf.py --preview set-target esp32c5 ; idf.py build
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32c5_ble)
