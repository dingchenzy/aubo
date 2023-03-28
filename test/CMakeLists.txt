cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(interface_mock_test)

set(CMAKE_CXX_STANDARD 17)

CPMImportPackage(json1)
CPMImportPackage(json-rpc-cxx1)
CPMImportPackage(googletest)

FILE(GLOB testSRC "*.cpp" "*.h" "*.hpp")

add_executable(interface_mock_test ${testSRC})
target_link_libraries(interface_mock_test
    common_interface
    gtest gmock gmock_main
    nlohmann_json::nlohmann_json
    json-rpc-cxx)
target_compile_definitions(interface_mock_test PRIVATE ENABLE_JSON_TYPES)
