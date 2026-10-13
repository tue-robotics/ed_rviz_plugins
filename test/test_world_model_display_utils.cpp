#include "displays/world_model_display_utils.h"

#include <gtest/gtest.h>

#include <regex>
#include <string>
#include <vector>

using ed_rviz_plugins::djb2;
using ed_rviz_plugins::split;
using ed_rviz_plugins::stringsToRegexes;

TEST(djb2, EmptyStringIsSeed)
{
    EXPECT_EQ(djb2(""), 5381U);
}

TEST(djb2, SingleCharacter)
{
    EXPECT_EQ(djb2("a"), 5381U * 33U + 'a');
}

// Reference values from the original signed int implementation, compiled with -fwrapv. Entity colors are picked with
// djb2(id) % 27, so these must not change.
TEST(djb2, MatchesOriginalImplementation)
{
    EXPECT_EQ(djb2("floor"), 259122023U);
    EXPECT_EQ(djb2("dinner_table"), 1479055756U);
    EXPECT_EQ(djb2("\xc3\xa9"), 5857809U); // "é", negative chars
}

TEST(djb2, NegativeSignedHashIsMadePositive)
{
    // As a signed 32-bit int, the hashes of these ids are negative
    EXPECT_EQ(djb2("cabinet"), 1309517925U);
    EXPECT_EQ(djb2("entity_1"), 612990286U);
}

TEST(split, SplitsOnDelimiter)
{
    EXPECT_EQ(split("a;bc;d", ';'), (std::vector<std::string>{"a", "bc", "d"}));
}

TEST(split, DropsEmptyItems)
{
    EXPECT_EQ(split(";a;;b;", ';'), (std::vector<std::string>{"a", "b"}));
    EXPECT_TRUE(split(";;;", ';').empty());
    EXPECT_TRUE(split("", ';').empty());
}

TEST(split, OtherDelimiter)
{
    EXPECT_EQ(split("a;b,c", ','), (std::vector<std::string>{"a;b", "c"}));
}

TEST(stringsToRegexes, EmptyStringGivesNoRegexes)
{
    EXPECT_TRUE(stringsToRegexes("").empty());
}

TEST(stringsToRegexes, MatchesWholeStringCaseInsensitive)
{
    const auto regexes = stringsToRegexes("table.*;Chair");
    ASSERT_EQ(regexes.size(), 2U);

    EXPECT_TRUE(std::regex_match("table", regexes[0]));
    EXPECT_TRUE(std::regex_match("TABLE_1", regexes[0]));
    EXPECT_FALSE(std::regex_match("dinner_table", regexes[0]));

    EXPECT_TRUE(std::regex_match("chair", regexes[1]));
    EXPECT_FALSE(std::regex_match("chairs", regexes[1]));
}

TEST(stringsToRegexes, InvalidPatternThrows)
{
    EXPECT_THROW(stringsToRegexes("valid;[invalid"), std::regex_error);
}
