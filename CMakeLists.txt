cmake_minimum_required(VERSION 3.16)
project(SoapyKA9Q VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SoapySDR CONFIG REQUIRED)
find_package(Threads REQUIRED)

SOAPY_SDR_MODULE_UTIL(
    TARGET ka9qSupport
    SOURCES SoapyKA9Q.cpp
    LIBRARIES Threads::Threads
)

if(APPLE)
    target_compile_definitions(ka9qSupport PRIVATE _DARWIN_C_SOURCE)
else()
    target_compile_definitions(ka9qSupport PRIVATE _DEFAULT_SOURCE)
endif()

