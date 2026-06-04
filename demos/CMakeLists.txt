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

cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

project(demos LANGUAGES CXX)

libhal_build_demos(
  DEMOS
  drc_v2
  drc_v2_resources
  dynamixel_scan
  mc_x_v2
  mc_x_v2_v5
  mc_x_v2_resources
  rc_servo
  rc_servo16
  rx_64
  rx_64_dual

  INCLUDES
  .

  PACKAGES
  libhal-actuator
  libhal-expander

  LINK_LIBRARIES
  libhal::actuator
  libhal::expander
)
