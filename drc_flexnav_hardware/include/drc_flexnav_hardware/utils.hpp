#include <type_traits>
#include <unordered_map>

template <typename T, typename U>
bool update_map_if_key_exists(std::unordered_map<T, U> &map, const T &key,
                              U new_value) {
  auto it = map.find(key);
  if (it != map.end()) {
    it->second = new_value;
    return true;
  }
  return false;
}

template <typename T, typename U>
bool mapped_value_is_valid(const std::unordered_map<T, U> &map, const T &key) {
  auto it = map.find(key);
  if (it != map.end()) {
    if (!std::is_floating_point_v<U>)
      return true;
    if (!std::isnan(it->second))
      return true;
  }
  return false;
}
