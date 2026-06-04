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

#include <libhal-exceptions/control.hpp>
#include <libhal-micromod/micromod.hpp>
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
  hal::micromod::v1::initialize_platform();
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
  hal::micromod::v1::reset();
}

void sleep(hal::time_duration p_duration)
{
  auto delay_clock = resources::clock();
  hal::delay(*delay_clock, p_duration);
}

hal::v5::strong_ptr<hal::steady_clock> clock()
{
  throw hal::operation_not_supported(nullptr);
  // return hal::micromod::v2::uptime_clock();
}

hal::v5::strong_ptr<hal::serial> console()
{
  throw hal::operation_not_supported(nullptr);
  // return hal::micromod::v2::console(hal::buffer<128>);
}

hal::v5::strong_ptr<hal::output_pin> status_led()
{
  throw hal::operation_not_supported(nullptr);
  // return hal::micromod::v2::led();
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  throw hal::operation_not_supported(nullptr);
  // static std::array<hal::can_message, 4> receive_buffer{};
  // return hal::micromod::v2::can_transceiver(receive_buffer);
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  throw hal::operation_not_supported(nullptr);
  // return hal::micromod::v2::can_bus_manager();
}

hal::v5::strong_ptr<hal::can_identifier_filter> can_identifier_filter()
{
  throw hal::operation_not_supported(nullptr);
  // return hal::micromod::v2::can_identifier_filter0();
}

hal::v5::strong_ptr<hal::pwm> pwm()
{
  throw hal::operation_not_supported(nullptr);
  // return hal::micromod::v2::pwm();
}

hal::v5::strong_ptr<hal::serial> uart2()
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
