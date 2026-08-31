cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_field_base)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/field.cpp
      src/data_loader.cpp
      src/operator.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/field/data_loader.h
        include/stepit/field/field.h
        include/stepit/field/operator.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC stepit_core)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

add_executable(data_info app/data_info.cpp)
target_link_libraries(data_info PRIVATE ${PROJECT_NAME} Boost::program_options)
install(TARGETS data_info
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)
