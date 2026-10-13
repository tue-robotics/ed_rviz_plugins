#ifndef WORLD_MODEL_DISPLAY_UTILS_H
#define WORLD_MODEL_DISPLAY_UTILS_H

#include <regex>
#include <string>
#include <vector>

namespace ed_rviz_plugins
{

/**
 * @brief djb2 hash of a string, used to give each entity a stable color
 * @return The absolute value of the hash, interpreted as a signed 32-bit integer
 */
unsigned int djb2(const std::string& str);

/**
 * @brief Split a string on a delimiter, dropping empty items
 */
std::vector<std::string> split(const std::string& str_to_split, char delimiter);

/**
 * @brief Convert a delimiter-separated list of patterns to case-insensitive regexes
 * @throws std::regex_error if one of the patterns is not a valid regex
 */
std::vector<std::regex> stringsToRegexes(const std::string& str_to_split, char delimiter = ';');

} // namespace ed_rviz_plugins

#endif // WORLD_MODEL_DISPLAY_UTILS_H
