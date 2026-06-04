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

  hal::print(*console, "Dynamixel Dual Demo Starting...\n");

  hal::actuator::rx_64::config servo1_config = {
    .baud_rate = 57600, .id = 4, .min_angle = 0, .max_angle = 230
  };

  hal::actuator::rx_64::config servo2_config = {
    .baud_rate = 57600, .id = 2, .min_angle = 70, .max_angle = 230
  };

  auto servo1 = hal::actuator::rx_64(uart, servo1_config, clock);
  auto servo2 = hal::actuator::rx_64(uart, servo2_config, clock);

  auto min_angle = servo1.min_angle();
  hal::print<32>(*console, "\nMin Angle: %.2f", min_angle);
  hal::delay(*clock, 10ms);

  auto max_angle = servo1.max_angle();
  hal::print<32>(*console, "\nMax Angle: %.2f", max_angle);
  hal::delay(*clock, 10ms);

  auto torque_limit = servo1.torque_limit();
  hal::print<32>(*console, "\nTorque Limit: %.2f", torque_limit);
  hal::delay(*clock, 10ms);

  auto temp_limit = servo1.temperature_limit();
  hal::print<32>(*console, "\nTemp Limit: %d", temp_limit);
  hal::delay(*clock, 10ms);

  auto min_voltage = servo1.min_voltage();
  hal::print<32>(*console, "\nMin Volt: %.2f", min_voltage);
  hal::delay(*clock, 10ms);

  auto max_voltage = servo1.max_voltage();
  hal::print<32>(*console, "\nMax Volt: %.2f", max_voltage);
  hal::delay(*clock, 10ms);

  auto return_delay = servo1.return_delay_time();
  hal::print<32>(*console, "\nReturn Delay: %d", return_delay);
  hal::delay(*clock, 10ms);

  auto punch = servo1.punch();
  hal::print<32>(*console, "\nPunch: %d", punch);
  hal::delay(*clock, 10ms);

  auto torque_enable = servo1.torque_enable();
  hal::print<32>(*console, "\nTorque Enable: %d", torque_enable);
  hal::delay(*clock, 10ms);

  auto moving_speed = servo1.moving_speed();
  hal::print<32>(*console, "\nRPMs: %.2f", moving_speed);
  hal::delay(*clock, 10ms);

  // loop and move servo
  while (true) {
    hal::print(*console, "\n70");
    servo1.sync_position(70, servo2);
    hal::delay(*clock, 3000ms);

    hal::print(*console, "\n150");
    servo1.sync_position(150, servo2);
    hal::delay(*clock, 3000ms);

    hal::print(*console, "\n230");
    servo1.sync_position(230, servo2);
    hal::delay(*clock, 3000ms);

    hal::print(*console, "\n150");
    servo1.sync_position(150, servo2);
    hal::delay(*clock, 3000ms);
  }
}
