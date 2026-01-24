// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <libhal-actuator/rc_servo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto pwm = resources::pwm_channel();
  auto pwm_frequency = resources::pwm_frequency();

  hal::print(*console, "RC Servo Application Starting...\n\n");

  hal::actuator::rc_servo16::settings rc_servo_settings{
    .frequency = 50,
    // Total 180 deg, change for your use case.
    .min_angle = -90,
    .max_angle = 90,
    // Change to 500us and 2500us if your rc servo
    // supports those pulse widths.
    .min_microseconds = 1000,
    .max_microseconds = 2000,
  };
  hal::actuator::rc_servo16 servo(*pwm_frequency, pwm, rc_servo_settings);

  hal::print(*console, "In 5 seconds...\n");
  hal::delay(*clock, 5000ms);

  // Oscillate from the servo horn back and forth
  while (true) {
    hal::print(*console, "-90 deg\n");
    servo.position(-90.0_deg);
    hal::delay(*clock, 1000ms);

    hal::print(*console, "0 deg\n");
    servo.position(0.0_deg);
    hal::delay(*clock, 1000ms);

    hal::print(*console, "90 deg\n");
    servo.position(90.0_deg);
    hal::delay(*clock, 1000ms);
  }
}
