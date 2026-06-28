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

#include <exception>

#include <libhal-actuator/smart_servo/rmd/drc_v2.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_identifier_filter = resources::can_identifier_filter();

  // Needs to be set to this baud rate to work with the default firmware CAN
  // baud rate.
  can_bus_manager->baud_rate(1.0_MHz);

  hal::print(*console, "RMD DRC Smart Servo Application Starting...\n\n");

  constexpr std::uint16_t starting_device_address = 0x140;
  std::uint16_t address_offset = 0;

  while (true) {
    try {
      auto const address = starting_device_address + address_offset;
      hal::actuator::rmd_drc_v2 drc(
        *can_transceiver, *can_identifier_filter, *clock, 6.0f, address);

      hal::print<128>(*console, "RMD DRC ID = 0x%02X\n", address);

      auto print_feedback = [&drc, &console]() {
        drc.feedback_request(hal::actuator::rmd_drc_v2::read::status_2);
        drc.feedback_request(
          hal::actuator::rmd_drc_v2::read::multi_turns_angle);
        drc.feedback_request(
          hal::actuator::rmd_drc_v2::read::status_1_and_error_flags);

        hal::print<2048>(
          *console,
          "[%u] =================================\n"
          "raw_multi_turn_angle = %f\n"
          "raw_current = %d\n"
          "raw_speed = %d\n"
          "raw_volts = %d\n"
          "encoder = %d\n"
          "raw_motor_temperature = %d"
          "\n"
          "over_voltage_protection_tripped = %d\n"
          "over_temperature_protection_tripped = %d\n"
          "-------\n"
          "angle() = %f °deg\n"
          "current() = %f A\n"
          "speed() = %f rpm\n"
          "volts() = %f V\n"
          "temperature() = %f °C\n\n",

          drc.feedback().message_number,
          static_cast<float>(drc.feedback().raw_multi_turn_angle),
          drc.feedback().raw_current,
          drc.feedback().raw_speed,
          drc.feedback().raw_volts,
          drc.feedback().encoder,
          drc.feedback().raw_motor_temperature,
          drc.feedback().over_voltage_protection_tripped(),
          drc.feedback().over_temperature_protection_tripped(),
          drc.feedback().angle(),
          drc.feedback().current(),
          drc.feedback().speed(),
          drc.feedback().volts(),
          drc.feedback().temperature());
      };

      hal::delay(*clock, 500ms);

      while (true) {
        drc.velocity_control(10.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        drc.velocity_control(-10.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        drc.position_control(0.0_deg, 5.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        drc.position_control(-45.0_deg, 5.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        drc.position_control(90.0_deg, 5.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        drc.position_control(180.0_deg, 5.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        drc.position_control(-360.0_deg, 5.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        drc.position_control(0.0_deg, 5.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();
      }
    } catch (hal::timed_out const&) {
      hal::print(
        *console,
        "hal::timed_out exception! which means that the device did not "
        "respond. Moving to the next device address in the list.\n");
    } catch (hal::resource_unavailable_try_again const& p_error) {
      hal::print(*console, "hal::resource_unavailable_try_again\n");
      if (p_error.instance() == &can_transceiver) {
        hal::print(
          *console,
          "\n"
          "The CAN peripheral has received no acknowledgements from any other "
          "device on the bus. It appears as if the peripheral is not connected "
          "to a can network. This can happen if the baud rate is incorrect, "
          "the CAN transceiver is not functioning, or the devices on the bus "
          "are not responding."
          "\n"
          "Calling terminate!"
          "\n"
          "Consider powering down the system and checking all of your "
          "connections before restarting the application.");
        std::terminate();
      }
      // otherwise keep trying with other addresses
    } catch (...) {
      hal::print(*console, "Unknown exception caught in (...) block\n");
      throw;  // see if anyone else can handle the exception
    }

    address_offset = (address_offset + 1) % 8;
    hal::delay(*clock, 1s);
  }
}
