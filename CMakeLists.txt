# Copyright 2024 - 2025 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.15)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

project(libhal-actuator LANGUAGES CXX)

libhal_test_and_make_library(
  LIBRARY_NAME libhal-actuator

  SOURCES
  src/rc_servo.cpp
  src/rx_64.cpp
  src/smart_servo/rmd/drc_v2.cpp
  src/smart_servo/rmd/mc_x_v2.cpp

  TEST_SOURCES
  tests/main.test.cpp
  tests/rc_servo.test.cpp
  tests/rx_64.test.cpp
  tests/smart_servo/rmd/drc.test.cpp
  tests/smart_servo/rmd/mc_x.test.cpp

  PACKAGES
  libhal-mock

  LINK_LIBRARIES
  libhal::mock
)
