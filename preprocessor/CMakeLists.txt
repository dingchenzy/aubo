cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(common_interface_generator)

set(CMAKE_CXX_STANDARD 17)

if(UNIX)
MANUAL_ADD_LUA_TARGET()
CPMImportPackage(sol3_1)
endif()
CPMImportPackage(json1)
CPMImportPackage(json-rpc-cxx1)
CPMImportPackage(asio)

file(GLOB SRC "*.cpp" "*.h" "*.hpp")

file(GLOB_RECURSE INTERFACE_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/*.hpp")

add_executable(common_interface_generator ${SRC})
target_link_libraries(common_interface_generator PRIVATE common_interface nlohmann_json::nlohmann_json json-rpc-cxx)
if(UNIX)
    target_link_libraries(common_interface_generator PRIVATE sol2::sol2 lua -lpthread)
endif()

target_compile_definitions(common_interface_generator PRIVATE ENABLE_JSON_TYPES)
set_target_properties(common_interface_generator PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/generated/aubo/
    )

file(MAKE_DIRECTORY
    ${CMAKE_BINARY_DIR}/generated/aubo/error_code
    ${CMAKE_BINARY_DIR}/generated/aubo/rtde_recipe)

if(CMAKE_CROSSCOMPILING)
    add_custom_target(common_interface_generator_run
        COMMAND qemu-aarch64 common_interface_generator
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/generated/aubo/
        COMMENT "Run generated common_interface_generator in ${CMAKE_BINARY_DIR}/generated/aubo/"
        DEPENDS ${INTERFACE_HEADERS}
    )
else()
    add_custom_target(common_interface_generator_run
        COMMAND common_interface_generator
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/generated/aubo/
        COMMENT "Run generated common_interface_generator in ${CMAKE_BINARY_DIR}/generated/aubo/"
        DEPENDS ${INTERFACE_HEADERS}
    )
endif()

macro(generateErrorCodeTranlator)
  cmake_parse_arguments(_gen "" "TARGET;OUT" "" "" ${ARGN})
  if(_gen_TARGET)
    add_dependencies(${_gen_TARGET} common_interface_generator_run)
  endif()

  #target_include_directories(${_gen_TARGET} PUBLIC ${CMAKE_BINARY_DIR}/generated)
  install(FILES
          ${CMAKE_BINARY_DIR}/generated/aubo/error_code/error_code.md
          ${CMAKE_BINARY_DIR}/generated/aubo/rtde_recipe/rtde_recipe.md
          DESTINATION docs/)
endmacro()
