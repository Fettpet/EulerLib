#include <Euler/Euler.h>
#include <algorithm>
#include <gtest/gtest.h>

using Euler::Set::Powerset;

TEST(Powerset, empty) {
    Powerset test;
    auto powSet = test(std::set<int>{});
    EXPECT_EQ(powSet.size(), 1);
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{}));
}

TEST(Powerset, oneElement) {
    Powerset test;
    auto powSet = test(std::set<int>{1});
    EXPECT_EQ(powSet.size(), 2);
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{1}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{}));
}

TEST(Powerset, twoElements) {
    Powerset test;
    auto powSet = test(std::set<int>{1, 2});
    EXPECT_EQ(powSet.size(), 4);
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{1}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{2}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{1, 2}));
}

TEST(Powerset, threeElements) {
    Powerset test;
    auto powSet = test(std::set<int>{1, 2, 3});
    EXPECT_EQ(powSet.size(), 8);
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{1}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{2}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{1, 2}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{1, 3}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{3}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{2, 3}));
    EXPECT_TRUE(std::count(powSet.begin(), powSet.end(), std::set<int>{1, 2, 3}));
}
