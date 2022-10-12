cmake_minimum_required(VERSION 3.1)
project(opensw VERSION 0.1.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Asio REQUIRED)
find_package(asyncapi_gencpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED core)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)

asyncapi_gencpp(${PROJECT_SOURCE_DIR}/api/asyncapi.yaml ${PROJECT_NAME}/msg ${CMAKE_CURRENT_BINARY_DIR}/include)

include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${ASIO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/base64.cpp
    src/client.cpp
    src/protocol.cpp)
target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBRARIES}
    spdlog::spdlog_header_only 
    Threads::Threads)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(opensw_cli src/bin/opensw_cli.cpp)
target_link_libraries(opensw_cli ${PROJECT_NAME})
set_property(TARGET opensw_cli PROPERTY CXX_STANDARD 17)

# Tests
enable_testing()
find_package(GTest REQUIRED)

add_executable(scan_data_tests test/scan_data_tests.cpp)
target_link_libraries(scan_data_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_definitions(scan_data_tests PUBLIC DATA_DIR="${PROJECT_SOURCE_DIR}/test/data/")
add_test(ScanDataTests scan_data_tests)

add_executable(map_data_tests test/map_data_tests.cpp)
target_link_libraries(map_data_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
set_property(TARGET map_data_tests PROPERTY CXX_STANDARD 17)
target_compile_definitions(map_data_tests PUBLIC DATA_DIR="${PROJECT_SOURCE_DIR}/test/data/")
add_test(MapDataTests map_data_tests)


install(
    TARGETS ${PROJECT_NAME} opensw_cli
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/msg
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
)
install(FILES package.xml DESTINATION share/opensw)

# pkg-config
set(PKG_CONFIG_C_FLAGS "-I${OpenCV_INCLUDE_DIRS} -I${EIGEN3_INCLUDE_DIRS}")
configure_file(pkg_config/opensw.pc.in ${CMAKE_CURRENT_BINARY_DIR}/opensw.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/opensw.pc DESTINATION lib/pkgconfig)