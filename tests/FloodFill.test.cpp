#include "Algorithm/FloodFill.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <array>
#include <iostream>

struct FloodFillFixture : public ::testing::Test {
    using Node = int;
    static constexpr uint32_t numberNodes = 10;
    using Graph = std::array<Node, numberNodes>;
    struct IsPart {
        [[nodiscard]] auto isPart(auto const& graph, auto const& node) const -> bool {
            return node >= 0 && node < static_cast<int>(graph.size());
        };
    };

    struct GetNeighbors {
        [[nodiscard]] auto getNeighbors(auto const& /*unused*/, auto const& node) const -> std::vector<int> {
            return std::vector<int>{node - 2, node + 2};
        };
    };

    FloodFillFixture()
        : floodFill(graph) {}

    Graph graph = Graph{};
    Algorithm::FloodFill<Graph, Node, IsPart, GetNeighbors> floodFill;
};

TEST_F(FloodFillFixture, eachSecondOdd) {
    auto constexpr startNode = 5;
    auto result = floodFill.calculate(startNode);
    std::sort(result.begin(), result.end());
    EXPECT_EQ(result, (std::vector<Node>{1, 3, 5, 7, 9}));
}

TEST_F(FloodFillFixture, eachSecondEven) {
    auto constexpr startNode = 2;
    auto result = floodFill.calculate(startNode);
    std::sort(result.begin(), result.end());
    EXPECT_EQ(result, (std::vector<Node>{0, 2, 4, 6, 8}));
}

TEST_F(FloodFillFixture, checkItsEmptyForSecondCall) {
    auto constexpr startNode = 2;
    (void)floodFill.calculate(startNode);
    auto result = floodFill.calculate(startNode);
    std::sort(result.begin(), result.end());
    EXPECT_EQ(result, (std::vector<Node>{}));
}

TEST_F(FloodFillFixture, checkClear) {
    auto constexpr startNode = 2;
    (void)floodFill.calculate(startNode);
    floodFill.clearVisited();
    auto result = floodFill.calculate(startNode);
    std::sort(result.begin(), result.end());
    EXPECT_EQ(result, (std::vector<Node>{0, 2, 4, 6, 8}));
}
