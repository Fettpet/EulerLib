#include <Euler/Euler.h>
#include <algorithm>
#include <gtest/gtest.h>

struct BreadthFirstFixture : public ::testing::Test {
    using Node = int;
    struct Graph {
        std::map<Node, std::vector<Node>> edgeList;

        void addEdge(Node const& n1, Node const& n2) { edgeList[n1].push_back(n2); }

        auto getEdges(Node const& n1) -> std::vector<Node> const { return edgeList[n1]; }
    };

    void SetUp() override {
        /**
         *  1
         *  |->2
         *  | |->3
         *  | |->4
         *  | |  |->5
         *  |->6
         * */
        graph.addEdge(1, 2);
        graph.addEdge(1, 6);
        graph.addEdge(2, 3);
        graph.addEdge(2, 4);
        graph.addEdge(4, 5);
    }

    class Visitor {
    public:
        virtual void init(Graph&, Node&) {}
        virtual void goOn(Graph&) {}

        virtual void beforeMove(Graph&, Node&, Node&) {}
        virtual void afterMove(Graph&, Node& node, Node&) { path.push_back(node); }

        virtual bool isSolveable() { return true; }

        virtual void shutdown() {}

        std::vector<Node> path;
    };

    auto getNeighborsFunctor() {
        return [](Graph& graph, Node& node) { return graph.getEdges(node); };
    }

    auto getIsSolvedFunctor(Node const& goal) {
        return [goal](Graph&, Node& node) { return goal == node; };
    }

    auto getMoveForwardFunctor() {
        return [&](Graph&, Node& node, Node& neigbor) {
            node = neigbor;
            return true;
        };
    }

    using BreadthFirstSearch = Euler::Graph::
        BreadthFirstSearch<BreadthFirstFixture::Graph, BreadthFirstFixture::Node, BreadthFirstFixture::Node>;
    Graph graph;
};

TEST_F(BreadthFirstFixture, check_level_order) {
    auto bfs = BreadthFirstFixture::BreadthFirstSearch{};
    auto visitor = BreadthFirstFixture::Visitor{};
    auto node = BreadthFirstFixture::Node{1};

    auto result =
        bfs.start(graph, node, visitor, getNeighborsFunctor(), getIsSolvedFunctor(5), getMoveForwardFunctor());

    // In this small graph, BFS will visit nodes in level order starting at 1
    EXPECT_TRUE(result); // 5 is reachable and should be detected by BFS

    auto visited = bfs.buildPath();
    // visited should contain start and then its neighbors in order they were enqueued
    // start was 1, then 2 and 6, then neighbors of 2 (3,4), etc.
    ASSERT_GE(visited.size(), 1u);
    EXPECT_EQ(visited[0], 1);
    // ensure uniqueness: no duplicate 2
    auto count2 = std::count(visited.begin(), visited.end(), 2);
    EXPECT_EQ(count2, 1);
}

TEST_F(BreadthFirstFixture, start_is_goal) {
    auto bfs = BreadthFirstFixture::BreadthFirstSearch{};
    auto visitor = BreadthFirstFixture::Visitor{};
    auto node = BreadthFirstFixture::Node{1};

    // isSolved on start should be detected
    auto result =
        bfs.start(graph, node, visitor, getNeighborsFunctor(), getIsSolvedFunctor(1), getMoveForwardFunctor());
    EXPECT_TRUE(result);
}

TEST_F(BreadthFirstFixture, unsolvable_goal) {
    auto bfs = BreadthFirstFixture::BreadthFirstSearch{};
    auto visitor = BreadthFirstFixture::Visitor{};
    auto node = BreadthFirstFixture::Node{4};

    auto result =
        bfs.start(graph, node, visitor, getNeighborsFunctor(), getIsSolvedFunctor(6), getMoveForwardFunctor());

    EXPECT_FALSE(result);
}

TEST_F(BreadthFirstFixture, no_duplicate_visits_for_shared_neighbors) {
    // Create a graph where node 4 is a neighbor of both 2 and 3
    // 1 -> {2,3}
    // 2 -> {4}
    // 3 -> {4}
    BreadthFirstFixture::Graph g;
    g.addEdge(1, 2);
    g.addEdge(1, 3);
    g.addEdge(2, 4);
    g.addEdge(3, 4);

    auto bfs = BreadthFirstFixture::BreadthFirstSearch{};
    auto visitor = BreadthFirstFixture::Visitor{};
    auto start = BreadthFirstFixture::Node{1};

    auto result = bfs.start(
        g,
        start,
        visitor,
        getNeighborsFunctor(),
        getIsSolvedFunctor(999), // unreachable
        getMoveForwardFunctor());

    EXPECT_FALSE(result);
    auto visited = bfs.buildPath();
    // ensure node 4 appears exactly once even though it's a neighbor of 2 and 3
    auto count4 = std::count(visited.begin(), visited.end(), 4);
    EXPECT_EQ(count4, 1);
}

TEST_F(BreadthFirstFixture, BreadthFirst_UnreachableGoal) {
    // two components: {1,2} and {3,4}
    BreadthFirstFixture::Graph g;
    g.addEdge(1, 2);
    g.addEdge(3, 4);

    auto bfs = BreadthFirstFixture::BreadthFirstSearch{};
    auto visitor = BreadthFirstFixture::Visitor{};
    auto start = BreadthFirstFixture::Node{1};

    auto result = bfs.start(g, start, visitor, getNeighborsFunctor(), getIsSolvedFunctor(4), getMoveForwardFunctor());
    EXPECT_FALSE(result);

    auto visited = bfs.buildPath();
    // Only nodes in component containing start (1,2) should be visited -> 3 should NOT be visited
    EXPECT_EQ(std::find(visited.begin(), visited.end(), 3), visited.end());
}

TEST_F(BreadthFirstFixture, BreadthFirst_CycleHandling) {
    // Create a cycle 1->2->3->1 and a branch from 3->5 (goal)
    BreadthFirstFixture::Graph g;
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 1);
    g.addEdge(3, 5);

    auto bfs = BreadthFirstFixture::BreadthFirstSearch{};
    auto visitor = BreadthFirstFixture::Visitor{};
    auto start = BreadthFirstFixture::Node{1};

    auto result = bfs.start(g, start, visitor, getNeighborsFunctor(), getIsSolvedFunctor(5), getMoveForwardFunctor());
    EXPECT_TRUE(result);

    auto visited = bfs.buildPath();
    // Nodes 1,2,3 should be visited exactly once despite the cycle
    EXPECT_EQ(std::count(visited.begin(), visited.end(), 1), 1);
    EXPECT_EQ(std::count(visited.begin(), visited.end(), 2), 1);
    EXPECT_EQ(std::count(visited.begin(), visited.end(), 3), 1);
}

TEST_F(BreadthFirstFixture, BreadthFirst_MoveForwardRejectsNeighbor) {
    // 1 -> {2,3}, 2 -> {4}, 3 -> {5(goal)}; we reject moves to 2
    BreadthFirstFixture::Graph g;
    g.addEdge(1, 2);
    g.addEdge(1, 3);
    g.addEdge(2, 4);
    g.addEdge(3, 5);

    auto bfs = BreadthFirstFixture::BreadthFirstSearch{};
    auto visitor = BreadthFirstFixture::Visitor{};
    auto start = BreadthFirstFixture::Node{1};

    // moveForward rejects neighbor == 2
    auto moveForwardReject2 =
        [&](BreadthFirstFixture::Graph&, BreadthFirstFixture::Node& node, BreadthFirstFixture::Node& neighbor) {
            if (neighbor == 2) return false;
            node = neighbor;
            return true;
        };

    auto result = bfs.start(g, start, visitor, getNeighborsFunctor(), getIsSolvedFunctor(5), moveForwardReject2);
    EXPECT_TRUE(result);

    auto visited = bfs.buildPath();
    // 2 should not be visited because moveForward rejected it
    EXPECT_EQ(std::count(visited.begin(), visited.end(), 2), 0);
    // 3 and 5 should be visited
    EXPECT_GE(std::count(visited.begin(), visited.end(), 3), 1);
    EXPECT_GE(std::count(visited.begin(), visited.end(), 5), 1);
}
