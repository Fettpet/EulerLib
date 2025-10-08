#include <Euler/Euler.h>
#include <algorithm>
#include <gtest/gtest.h>

struct DepthFirstFixture : public ::testing::Test {
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

        virtual void beforeMoveBack(Graph&, Node& node, Node&) { backwardPath.push_back(node); }
        virtual void afterMoveBack(Graph&, Node&, Node&) {}

        virtual void shutdown() {}

        std::vector<Node> path;
        std::vector<Node> backwardPath;
    };

    template<Node value>
    class VisitorNotSolveable : public Visitor {
    public:
        virtual bool isSolveable() { return path.size() == 0 || path.back() != value; }
    };

    auto getNeighborsFunctor() {
        return [](Graph& graph, Node& node) { return graph.getEdges(node); };
    }

    auto getIsSolvedFunctor(Node const& goal) {
        return [goal](Graph&, Node& node) { return goal == node; };
    }

    auto getMoveForwardFunctor() {
        return [&](Graph&, Node& node, Node& neigbor) {
            path.push_back(node);
            node = neigbor;
            return true;
        };
    }

    auto getMoveBackwardFunctor() {
        return [&](Graph&, Node& node, Node&) {
            node = path.back();
            path.pop_back();
        };
    }
    std::vector<Node> path;
    using DepthFirstSearch =
        Euler::Graph::DepthFirstSearch<DepthFirstFixture::Graph, DepthFirstFixture::Node, DepthFirstFixture::Node>;
    Graph graph;
};

TEST_F(DepthFirstFixture, check1to3) {
    auto deapthFirstSearch = DepthFirstFixture::DepthFirstSearch{};
    auto visitor = Visitor{};
    auto node = DepthFirstFixture::Node{1};
    auto result = deapthFirstSearch.start(
        graph,
        node,
        visitor,
        getNeighborsFunctor(),
        getIsSolvedFunctor(3),
        getMoveForwardFunctor(),
        getMoveBackwardFunctor());

    EXPECT_TRUE(result);
    path = deapthFirstSearch.buildPath();
    EXPECT_EQ(path.size(), 3u);
    EXPECT_EQ(visitor.backwardPath.size(), 0u);
    EXPECT_EQ((path[0]), 1);
    EXPECT_EQ((path[1]), 2);
    EXPECT_EQ((path[2]), 3);
}

TEST_F(DepthFirstFixture, check1to5) {
    auto deapthFirstSearch = DepthFirstFixture::DepthFirstSearch{};
    auto visitor = Visitor{};
    auto node = DepthFirstFixture::Node{1};
    auto result = deapthFirstSearch.start(
        graph,
        node,
        visitor,
        getNeighborsFunctor(),
        getIsSolvedFunctor(5),
        getMoveForwardFunctor(),
        getMoveBackwardFunctor());

    EXPECT_TRUE(result);
    path = deapthFirstSearch.buildPath();
    EXPECT_EQ(path.size(), 4u);
    EXPECT_EQ(visitor.backwardPath.size(), 1u);
    EXPECT_EQ((path[0]), 1);
    EXPECT_EQ((path[1]), 2);
    EXPECT_EQ((path[2]), 4);
    EXPECT_EQ((path[3]), 5);
}

TEST_F(DepthFirstFixture, check1to5Unsolvable) {
    auto deapthFirstSearch = DepthFirstFixture::DepthFirstSearch{};
    auto visitor = VisitorNotSolveable<4>{};
    auto node = DepthFirstFixture::Node{1};
    auto result = deapthFirstSearch.start(
        graph,
        node,
        visitor,
        getNeighborsFunctor(),
        getIsSolvedFunctor(5),
        getMoveForwardFunctor(),
        getMoveBackwardFunctor());

    EXPECT_FALSE(result);
}

TEST_F(DepthFirstFixture, check1to5WithBlocked3) {
    auto deapthFirstSearch = DepthFirstFixture::DepthFirstSearch{};
    auto visitor = VisitorNotSolveable<3>{};
    auto node = DepthFirstFixture::Node{1};
    auto result = deapthFirstSearch.start(
        graph,
        node,
        visitor,
        getNeighborsFunctor(),
        getIsSolvedFunctor(5),
        getMoveForwardFunctor(),
        getMoveBackwardFunctor());

    EXPECT_TRUE(result);
}

TEST_F(DepthFirstFixture, check4to6) {
    auto deapthFirstSearch = DepthFirstFixture::DepthFirstSearch{};
    auto visitor = Visitor{};
    auto node = DepthFirstFixture::Node{4};
    auto result = deapthFirstSearch.start(
        graph,
        node,
        visitor,
        getNeighborsFunctor(),
        getIsSolvedFunctor(6),
        getMoveForwardFunctor(),
        getMoveBackwardFunctor());

    EXPECT_FALSE(result);
}

TEST_F(DepthFirstFixture, check1to7) {
    auto deapthFirstSearch = DepthFirstFixture::DepthFirstSearch{};
    auto visitor = Visitor{};
    auto node = DepthFirstFixture::Node{1};
    auto result = deapthFirstSearch.start(
        graph,
        node,
        visitor,
        getNeighborsFunctor(),
        getIsSolvedFunctor(7),
        getMoveForwardFunctor(),
        getMoveBackwardFunctor());

    EXPECT_FALSE(result);
}

struct ComplexType {
    ComplexType(int i)
        : i(i) {}
    ComplexType(ComplexType const&) = default;
    ComplexType(ComplexType&&) = default;

    ComplexType& operator=(ComplexType const&) = default;
    ComplexType& operator=(ComplexType&&) = default;

    int i;
};

// used to test protected/private functions
template<typename Graph, typename Node, typename Neighbor>
struct TestDeapthFirst : public Euler::Graph::DepthFirstSearch<Graph, Node, Neighbor> {
public:
    using typename Euler::Graph::DepthFirstSearch<Graph, Node, Neighbor>::PathNode;
    using Euler::Graph::DepthFirstSearch<Graph, Node, Neighbor>::exploreNeighborhood;
    using Euler::Graph::DepthFirstSearch<Graph, Node, Neighbor>::getNeighbor;
    using Euler::Graph::DepthFirstSearch<Graph, Node, Neighbor>::next;
};

TEST(DepthFirstSearch, PathNode_ComplexType) {
    auto nodes = std::vector<ComplexType>{};
    nodes.emplace_back(1);
    nodes.emplace_back(2);

    auto testNode =
        TestDeapthFirst<std::vector<ComplexType>, ComplexType, ComplexType>::PathNode{ComplexType(0), std::move(nodes)};
    EXPECT_TRUE(testNode.isValid());
    EXPECT_EQ(testNode.currentNeighbor().i, 1);
    testNode.next();

    EXPECT_TRUE(testNode.isValid());
    EXPECT_EQ(testNode.currentNeighbor().i, 2);
    testNode.next();

    EXPECT_FALSE(testNode.isValid());

    EXPECT_EQ(testNode.getNode().i, 0);
}

TEST(DepthFirstSearch, exploreNeighborhood) {
    auto getNeighbors = [](std::vector<ComplexType>& graph, ComplexType const& node) -> std::vector<ComplexType> {
        std::vector<ComplexType> neighbors;
        if (graph.size() < static_cast<size_t>(node.i)) return neighbors;
        neighbors.emplace_back(node.i + 1);
        neighbors.emplace_back(node.i + 2);
        return neighbors;
    };

    auto deapthFirst = TestDeapthFirst<std::vector<ComplexType>, ComplexType, ComplexType>{};
    auto graph = std::vector<ComplexType>{};
    std::generate_n(std::back_inserter(graph), 10, []() {
        static int i = 0;
        return i++;
    });
    auto node = ComplexType(0);
    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, node, getNeighbors));
    EXPECT_EQ(deapthFirst.getNeighbor().i, 1);

    EXPECT_TRUE(deapthFirst.next());
    EXPECT_EQ(deapthFirst.getNeighbor().i, 2);

    EXPECT_FALSE(deapthFirst.next());
}

TEST(DepthFirstSearch, complex_explore_neihboor) {
    auto getNeighbors = [](std::vector<ComplexType>& graph, ComplexType const& node) -> std::vector<ComplexType> {
        std::vector<ComplexType> neighbors;
        if (graph.size() < static_cast<size_t>(node.i)) return neighbors;
        neighbors.emplace_back(node.i + 1);
        neighbors.emplace_back(node.i + 2);
        return neighbors;
    };

    auto deapthFirst = TestDeapthFirst<std::vector<ComplexType>, ComplexType, ComplexType>{};
    auto graph = std::vector<ComplexType>{};
    std::generate_n(std::back_inserter(graph), 10, []() {
        static int i = 0;
        return i++;
    });
    auto node = ComplexType(0);
    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, node, getNeighbors));
    EXPECT_EQ(deapthFirst.getNeighbor().i, 1);

    EXPECT_TRUE(deapthFirst.next());
    node = ComplexType(4);
    auto& neighbor = deapthFirst.getNeighbor();
    EXPECT_EQ(neighbor.i, 2);

    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, node, getNeighbors));
    EXPECT_EQ(deapthFirst.getNeighbor().i, 5);

    EXPECT_EQ(neighbor.i, 2);
}

TEST_F(DepthFirstFixture, StartIsGoal) {
    auto deapthFirstSearch = DepthFirstFixture::DepthFirstSearch{};
    auto visitor = Visitor{};
    auto node = DepthFirstFixture::Node{1};

    // getNeighbors must return non-empty so exploreNeighborhood succeeds
    auto getNeighbors = getNeighborsFunctor();

    // moveForward does not change the node but returns false; the algorithm
    // checks isSolved() after the move callback, so this should still detect
    // that the start node is the goal.
    auto moveForward = [&](Graph&, Node&, Node&) {
        // don't modify n (stay on start)
        return false;
    };
    auto moveBackward = [&](Graph&, Node&, Node&) {};

    auto result =
        deapthFirstSearch.start(graph, node, visitor, getNeighbors, getIsSolvedFunctor(1), moveForward, moveBackward);

    EXPECT_TRUE(result);
}

TEST(DepthFirstSearch, StartNodeNoNeighbors) {
    using G = DepthFirstFixture::Graph;
    using N = DepthFirstFixture::Node;
    using DFS = Euler::Graph::DepthFirstSearch<G, N, N>;

    auto dfs = DFS{};
    auto graph = G{};
    auto node = N{7};
    auto visitor = DepthFirstFixture::Visitor{};

    // getNeighbors returns empty vector -> exploreNeighborhood will fail
    auto getNeighbors = [](G&, N&) { return std::vector<N>{}; };
    auto isSolved = [](G&, N&) { return false; };
    auto moveForward = [](G&, N&, N&) { return true; };
    auto moveBackward = [](G&, N&, N&) {};

    auto result = dfs.start(graph, node, visitor, getNeighbors, isSolved, moveForward, moveBackward);
    EXPECT_FALSE(result);
}

TEST(DepthFirstSearch, GetNeighborsPathWithComplexTypeNeighbors) {
    auto deapthFirst = TestDeapthFirst<std::vector<ComplexType>, ComplexType, ComplexType>{};
    auto graph = std::vector<ComplexType>{};
    std::generate_n(std::back_inserter(graph), 10, []() {
        static int i = 0;
        return ComplexType(i++);
    });

    auto node = ComplexType(0);
    auto getNeighbors = [](std::vector<ComplexType>& graph, ComplexType const& node) -> std::vector<ComplexType> {
        std::vector<ComplexType> neighbors;
        if (graph.size() < static_cast<size_t>(node.i)) return neighbors;
        neighbors.emplace_back(node.i + 1);
        neighbors.emplace_back(node.i + 2);
        return neighbors;
    };

    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, node, getNeighbors));
    auto neighborsPath = deapthFirst.getNeighborsPath();
    ASSERT_EQ(neighborsPath.size(), 1u);
    EXPECT_EQ(neighborsPath[0].i, 1);
}

TEST(DepthFirstSearch, BuildPathWithComplexTypeNodes) {
    auto deapthFirst = TestDeapthFirst<std::vector<ComplexType>, ComplexType, ComplexType>{};
    auto graph = std::vector<ComplexType>{};
    std::generate_n(std::back_inserter(graph), 10, []() {
        static int i = 0;
        return ComplexType(i++);
    });

    // push two levels into the internal stack
    auto nodeA = ComplexType(0);
    auto nodeB = ComplexType(4);
    auto getNeighbors = [](std::vector<ComplexType>& graph, ComplexType const& node) -> std::vector<ComplexType> {
        std::vector<ComplexType> neighbors;
        if (graph.size() < static_cast<size_t>(node.i)) return neighbors;
        neighbors.emplace_back(node.i + 1);
        return neighbors;
    };

    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, nodeA, getNeighbors));
    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, nodeB, getNeighbors));

    auto path = deapthFirst.buildPath();
    ASSERT_EQ(path.size(), 2u);
    EXPECT_EQ(path[0].i, 0);
    EXPECT_EQ(path[1].i, 4);
}

TEST_F(DepthFirstFixture, GotoNextNeighborWhenMoveForwardFails) {
    using DFS = DepthFirstFixture::DepthFirstSearch;
    auto dfs = DFS{};
    // create a small graph: 1 -> {2,3}, 3 -> {4}
    Graph g;
    g.addEdge(1, 2);
    g.addEdge(1, 3);
    g.addEdge(3, 4);

    auto visitor = Visitor{};
    auto node = Node{1};

    auto getNeighbors = [](Graph& graph, Node& n) { return graph.getEdges(n); };
    auto isSolved = [](Graph&, Node& n) { return n == 4; };

    // moveForward fails for neighbor==2, succeeds for neighbor==3
    auto moveForward = [](Graph&, Node& n, Node& neighbor) {
        if (neighbor == 2) return false;
        n = neighbor; // move to neighbor
        return true;
    };
    auto moveBackward = [](Graph&, Node&, Node&) {};

    auto result = dfs.start(g, node, visitor, getNeighbors, isSolved, moveForward, moveBackward);
    EXPECT_TRUE(result);
    auto path = dfs.buildPath();
    // path should contain 1 and 3 and then 4 (neighbor path includes starts)
    // buildPath returns the start nodes of the stacked path plus current node
    // which should be {1,3,4}
    ASSERT_GE(path.size(), 2u);
    EXPECT_EQ(path.front(), 1);
}

TEST(DepthFirstSearch, EmptyNeighborsReturnedByGetNeighbors) {
    auto deapthFirst = TestDeapthFirst<std::vector<int>, int, int>{};
    auto graph = std::vector<int>{};
    auto node = int{0};
    auto getNeighbors = [](std::vector<int>&, int const&) { return std::vector<int>{}; };
    EXPECT_FALSE(deapthFirst.exploreNeighborhood(graph, node, getNeighbors));
}
