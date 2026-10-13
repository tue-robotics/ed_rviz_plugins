#include "world_model_display_utils.h"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace ed_rviz_plugins
{

unsigned int djb2(const std::string& str)
{
    // Unsigned arithmetic wraps where the original signed int version overflowed (undefined behavior), while
    // producing the same bits, so entities keep their colors.
    std::uint32_t hash = 5381;
    for (const char c : str)
        hash = (hash * 33) + static_cast<std::uint32_t>(c);

    // Absolute value of the hash as a signed 32-bit integer
    if ((hash & 0x80000000U) != 0)
        hash = 0U - hash;

    return hash;
}

std::vector<std::string> split(const std::string& str_to_split, char delimiter)
{
    std::stringstream ss(str_to_split);
    std::string item;
    std::vector<std::string> result;
    while (std::getline(ss, item, delimiter))
    {
        if (!item.empty())
            result.push_back(item);
    }
    return result;
}

std::vector<std::regex> stringsToRegexes(const std::string& str_to_split, char delimiter)
{
    const auto strings = split(str_to_split, delimiter);
    std::vector<std::regex> regexes;
    regexes.reserve(strings.size());
    std::ranges::transform(strings,
                           std::back_inserter(regexes),
                           [](const std::string& str) { return std::regex(str, std::regex_constants::icase); });
    return regexes;
}

} // namespace ed_rviz_plugins
