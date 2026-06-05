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

#include <libhal-actuator/mx_64.hpp>
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

  hal::print(*console, "Dynamixel Demo Starting...\n");

  hal::actuator::mx_64::config servo_config = {
    .baud_rate = 57600, .id = 4, .min_angle = 0, .max_angle = 360
  };

  auto servo = hal::actuator::mx_64(uart, servo_config, clock);

  auto min_angle = servo.min_angle();
  hal::print<32>(*console, "\nMin Angle: %.2f", min_angle);
  hal::delay(*clock, 10ms);

  auto max_angle = servo.max_angle();
  hal::print<32>(*console, "\nMax Angle: %.2f", max_angle);
  hal::delay(*clock, 10ms);

  auto torque_limit = servo.torque_limit();
  hal::print<32>(*console, "\nTorque Limit: %.2f", torque_limit);
  hal::delay(*clock, 10ms);

  auto temp_limit = servo.temperature_limit();
  hal::print<32>(*console, "\nTemp Limit: %d", temp_limit);
  hal::delay(*clock, 10ms);

  auto min_voltage = servo.min_voltage();
  hal::print<32>(*console, "\nMin Volt: %.2f", min_voltage);
  hal::delay(*clock, 10ms);

  auto max_voltage = servo.max_voltage();
  hal::print<32>(*console, "\nMax Volt: %.2f", max_voltage);
  hal::delay(*clock, 10ms);

  auto return_delay = servo.return_delay_time();
  hal::print<32>(*console, "\nReturn Delay: %d", return_delay);
  hal::delay(*clock, 10ms);

  auto punch = servo.punch();
  hal::print<32>(*console, "\nPunch: %d", punch);
  hal::delay(*clock, 10ms);

  auto torque_enable = servo.torque_enable();
  hal::print<32>(*console, "\nTorque Enable: %d", torque_enable);
  hal::delay(*clock, 10ms);

  auto moving_speed = servo.moving_speed();
  hal::print<32>(*console, "\nRPMs: %.2f", moving_speed);
  hal::delay(*clock, 10ms);

  // loop and move servo
  while (true) {
    hal::print(*console, "\n0");
    servo.position(0);
    hal::delay(*clock, 3000ms);

    hal::print(*console, "\n180");
    servo.position(180);
    hal::delay(*clock, 3000ms);

    hal::print(*console, "\n360");
    servo.position(360);
    hal::delay(*clock, 3000ms);

    hal::print(*console, "\n180");
    servo.position(180);
    hal::delay(*clock, 3000ms);
  }
}
