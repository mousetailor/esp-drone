# ESP32 Controller Firmware — Top-level CMake
# This ESP32 sits with the pilot, connected to PC via USB.
# It bridges PC (UART) <-> Drone ESP32 (ESP-NOW wireless).
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_controller)
