
#ifndef DRC_FLEXNAV_CONTROLLERS__UTILS_HPP_
#define DRC_FLEXNAV_CONTROLLERS__UTILS_HPP_

#include <concepts>
#include <iostream>
#include <limits>
#include <ranges>
#include <type_traits>

#include <tf2_msgs/msg/tf_message.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

template <typename A>
concept numeric_or_bool = std::is_arithmetic_v<A>;

template <typename T, typename U, typename V = T>
  requires numeric_or_bool<V>
T return_state_if_exists(
    const T & /*msg*/, const U &interface_name,
    const std::vector<hardware_interface::LoanedStateInterface> &interfaces,
    const V &default_error_value = static_cast<V>(0)) {
  for (const auto &interface : interfaces) {
    /*std::cerr << "interface.get_name(): " << interface.get_name() << " |
     * interface_name: " << interface_name << std::endl;*/
    if (interface.get_name() == interface_name) {
      return static_cast<T>(interface.get_value());
    }
  }
  if constexpr (std::is_floating_point_v<T>) {
    return std::numeric_limits<T>::quiet_NaN();
  }
  /* std::cerr << "Interface: " << interface_name << " is set to " */
  /*           << static_cast<T>(default_error_value) */
  /*           << ", but the correct value was not found because the interface "
   */
  /*              "was not accessible" */
  /*           << std::endl; */
  return static_cast<T>(default_error_value);
}

template <typename T, typename U>
bool set_command_if_exists(
    const T &value, const U &interface_name,
    std::vector<hardware_interface::LoanedCommandInterface> &interfaces) {

  for (auto &interface : interfaces) {
    if (interface.get_name() == interface_name) {
      interface.set_value(static_cast<double>(value));
      return true;
    }
  }
  return false;
}

template <std::ranges::range T, std::ranges::range U>
bool contains_subcontainer(T container, U subcontainer) {
  if (std::ranges::empty(subcontainer) ||
      std::ranges::size(subcontainer) > std::ranges::size(container)) {
    return false;
  }

  auto it = std::search(
      std::ranges::begin(container), std::ranges::end(container),
      std::ranges::begin(subcontainer), std::ranges::end(subcontainer));

  return it != std::ranges::end(container);
}

// Print array
template <class T, std::size_t N>
std::ostream &operator<<(std::ostream &o, const std::array<T, N> &arr) {
  std::copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
  return o;
}

double getLongitudinalVelocity(const double& vx, const double& vy, const double& theta);

#endif // DRC_FLEXNAV_CONTROLLERS__UTILS_HPP_
