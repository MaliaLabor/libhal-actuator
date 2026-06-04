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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/i2c.hpp>
#include <libhal-arm-mcu/lpc40/output_pin.hpp>
#include <libhal-arm-mcu/lpc40/pwm.hpp>
#include <libhal-arm-mcu/lpc40/uart.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>

#include <resource_list.hpp>

hal::v5::optional_ptr<hal::steady_clock> clock_ptr;
hal::v5::optional_ptr<hal::serial> console_ptr;
hal::v5::optional_ptr<hal::output_pin> status_led_ptr;

[[noreturn]] void terminate_handler() noexcept
{
  if (not status_led_ptr && not clock_ptr) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern

  while (true) {
    using namespace std::chrono_literals;
    status_led_ptr->level(false);
    hal::delay(*clock_ptr, 100ms);
    status_led_ptr->level(true);
    hal::delay(*clock_ptr, 100ms);
    status_led_ptr->level(false);
    hal::delay(*clock_ptr, 100ms);
    status_led_ptr->level(true);
    hal::delay(*clock_ptr, 1000ms);
  }
}

void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(terminate_handler);
  // Set the MCU to the maximum clock speed
  hal::lpc40::maximum(12.0_MHz);
}

namespace resources {
using namespace hal::literals;
std::pmr::polymorphic_allocator<> driver_allocator()
{
  static std::array<hal::byte, 1024> driver_memory{};
  static std::pmr::monotonic_buffer_resource resource(
    driver_memory.data(),
    driver_memory.size(),
    std::pmr::null_memory_resource());
  return &resource;
}

void reset()
{
  hal::cortex_m::reset();
}

void sleep(hal::time_duration p_duration)
{
  auto delay_clock = resources::clock();
  hal::delay(*delay_clock, p_duration);
}

hal::v5::strong_ptr<hal::steady_clock> clock()
{
  if (clock_ptr) {
    return clock_ptr;
  }

  clock_ptr = hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(
    driver_allocator(), hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));
  return clock_ptr;
}

hal::v5::strong_ptr<hal::serial> console()
{
  static std::array<hal::byte, 64> uart0_buffer{};
  return hal::v5::make_strong_ptr<hal::lpc40::uart>(driver_allocator(),
                                                    0,
                                                    uart0_buffer,
                                                    hal::serial::settings{
                                                      .baud_rate = 115200,
                                                    });
}

hal::v5::strong_ptr<hal::output_pin> status_led()
{
  if (status_led_ptr) {
    return status_led_ptr;
  }

  status_led_ptr =
    hal::v5::make_strong_ptr<hal::lpc40::output_pin>(driver_allocator(), 1, 10);
  return status_led_ptr;
}

hal::v5::strong_ptr<hal::i2c> i2c()
{
  return hal::v5::make_strong_ptr<hal::lpc40::i2c>(driver_allocator(),
                                                   2,
                                                   hal::i2c::settings{
                                                     .clock_rate = 100.0_kHz,
                                                   });
}

hal::v5::strong_ptr<hal::serial> uart2()
{
  throw hal::operation_not_supported(nullptr);
}
hal::v5::strong_ptr<hal::pwm> pwm()
{
  return hal::v5::make_strong_ptr<hal::lpc40::pwm>(
    resources::driver_allocator(), 1, 6);
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  throw hal::operation_not_supported(nullptr);
}
hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  throw hal::operation_not_supported(nullptr);
}
hal::v5::strong_ptr<hal::can_identifier_filter> can_identifier_filter()
{
  throw hal::operation_not_supported(nullptr);
}
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel()
{
  throw hal::operation_not_supported(nullptr);
}
hal::v5::strong_ptr<hal::pwm_group_manager> pwm_frequency()
{
  throw hal::operation_not_supported(nullptr);
}
}  // namespace resources
