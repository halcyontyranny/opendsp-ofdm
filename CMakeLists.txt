cmake_minimum_required(VERSION 3.16)
project(opendsp_ofdm VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3    REQUIRED fftw3)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

option(WITH_HAMLIB "Enable Hamlib CAT/PTT support" ON)
if(WITH_HAMLIB)
  pkg_check_modules(HAMLIB hamlib)
  if(HAMLIB_FOUND)
    add_compile_definitions(HAVE_HAMLIB)
    message(STATUS "Hamlib found — CAT/PTT enabled")
  else()
    message(WARNING "Hamlib not found — building without CAT/PTT")
  endif()
endif()

# ── Library ───────────────────────────────────────────────────────────────────
set(LIB_SOURCES
    src/ofdm_core.cpp
    src/acm.cpp
    src/framer.cpp
    src/negotiation.cpp
    src/audio_io.cpp
)

add_library(opendsp_ofdm_lib STATIC ${LIB_SOURCES})
target_include_directories(opendsp_ofdm_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${FFTW3_INCLUDE_DIRS}
    ${PORTAUDIO_INCLUDE_DIRS}
)
target_link_libraries(opendsp_ofdm_lib PUBLIC
    ${FFTW3_LIBRARIES}
    ${PORTAUDIO_LIBRARIES}
)
if(WITH_HAMLIB AND HAMLIB_FOUND)
  target_include_directories(opendsp_ofdm_lib PUBLIC ${HAMLIB_INCLUDE_DIRS})
  target_link_libraries(opendsp_ofdm_lib PUBLIC ${HAMLIB_LIBRARIES})
endif()

# ── Executables ───────────────────────────────────────────────────────────────
add_executable(opendsp_ofdm src/main.cpp)
target_link_libraries(opendsp_ofdm PRIVATE opendsp_ofdm_lib)

add_executable(sim_test src/sim_test.cpp)
target_link_libraries(sim_test PRIVATE opendsp_ofdm_lib)

install(TARGETS opendsp_ofdm sim_test DESTINATION bin)
