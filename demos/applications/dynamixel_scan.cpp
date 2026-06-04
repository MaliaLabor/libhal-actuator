// Copyright 2026 Malia Labor and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <cstdint>

#include <libhal-actuator/rx_64.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto uart = resources::uart2();

  hal::print(*console, "Dynamixel Scan Application Starting...\n");
  hal::actuator::rx_64::config servo_config = {
    .baud_rate = 57600, .id = 4, .min_angle = 0, .max_angle = 300
  };
  auto servo = hal::actuator::rx_64(uart, servo_config, clock);

  std::array<hal::hertz, 9> available_bauds = { 9600,   19200,  57600,
                                                115200, 200000, 250000,
                                                400000, 500000, 1000000 };
  bool baud_rate_found = false;
  bool device_found = false;
  // loop through bauds
  for (auto current_baud : available_bauds) {
    uart->configure({ .baud_rate = current_baud });
    hal::print<48>(*console, "\nSending Pings on baud %f : ", current_baud);
    for (uint8_t i = 0; i < 254; i++) {
      hal::print(*console, ".");
      device_found = hal::actuator::rx_64::ping_id(i, uart, clock);
      if (device_found) {
        hal::print<64>(*console, "\nID found: %d \n", i);

        baud_rate_found = true;
      }
    }

    if (baud_rate_found) {
      break;
    }
  }
}
