cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_publisher_csv)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/publisher_csv.cpp
    INCLUDES include
    FLAGS -Wall -Wextra -Wpedantic -Wno-unused-parameter
)
