#include "Algorithm/DepthFirstSearch.h"
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

        virtual void beforeMove(Graph&, std::shared_ptr<Node>&, std::shared_ptr<Node>&) {}
        virtual void afterMove(Graph&, std::shared_ptr<Node>& node, std::shared_ptr<Node>&) { path.push_back(*node); }

        virtual bool isSolveable() { return true; }

        virtual void beforeMoveBack(Graph&, std::shared_ptr<Node>& node, std::shared_ptr<Node>&) {
            backwardPath.push_back(*node);
        }
        virtual void afterMoveBack(Graph&, std::shared_ptr<Node>&, std::shared_ptr<Node>&) {}

        virtual void shutdown() {}

        std::vector<Node> path;
        std::vector<Node> backwardPath;
    };

    template<Node value>
    class VisitorNotSolveable: public Visitor {
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
        return [&](Graph&, std::shared_ptr<Node>& node, std::shared_ptr<Node>& neigbor) {
            path.push_back(node);
            node = neigbor;
            return true;
        };
    }

    auto getMoveBackwardFunctor() {
        return [&](Graph&, std::shared_ptr<Node>& node, std::shared_ptr<Node>&) {
            node = path.back();
            path.pop_back();
        };
    }
    std::vector<std::shared_ptr<Node>> path;
    using DepthFirstSearch =
        Algorithm::DeapthFirstSearch<DepthFirstFixture::Graph, DepthFirstFixture::Node, DepthFirstFixture::Node>;
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
    EXPECT_EQ(path.size(), 3);
    EXPECT_EQ(visitor.backwardPath.size(), 0);
    EXPECT_EQ(*(path[0]), 1);
    EXPECT_EQ(*(path[1]), 2);
    EXPECT_EQ(*(path[2]), 3);
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
    EXPECT_EQ(path.size(), 4);
    EXPECT_EQ(visitor.backwardPath.size(), 1);
    EXPECT_EQ(*(path[0]), 1);
    EXPECT_EQ(*(path[1]), 2);
    EXPECT_EQ(*(path[2]), 4);
    EXPECT_EQ(*(path[3]), 5);
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

struct MoveOnly {
    MoveOnly(int i)
        : i(i) {}
    MoveOnly(MoveOnly const&) = delete;
    MoveOnly(MoveOnly&&) = default;

    MoveOnly& operator=(MoveOnly const&) = delete;
    MoveOnly& operator=(MoveOnly&&) = default;

    int i;
};

// used to test protected/private functions
template<typename Graph, typename Node, typename Neighbor>
struct TestDeapthFirst : public Algorithm::DeapthFirstSearch<Graph, Node, Neighbor> {
public:
    using typename Algorithm::DeapthFirstSearch<Graph, Node, Neighbor>::PathNode;
    using Algorithm::DeapthFirstSearch<Graph, Node, Neighbor>::exploreNeighborhood;
    using Algorithm::DeapthFirstSearch<Graph, Node, Neighbor>::getNeighbor;
    using Algorithm::DeapthFirstSearch<Graph, Node, Neighbor>::next;
};

TEST(DeapthFirstSearch, PathNode_MoveOnly) {
    auto nodes = std::vector<MoveOnly>{};
    nodes.emplace_back(1);
    nodes.emplace_back(2);

    auto testNode = TestDeapthFirst<std::vector<MoveOnly>, MoveOnly, MoveOnly>::PathNode{
        std::make_shared<MoveOnly>(0), std::move(nodes)};
    EXPECT_TRUE(testNode.isValid());
    EXPECT_EQ(testNode.currentNeighbor()->i, 1);
    testNode.next();

    EXPECT_TRUE(testNode.isValid());
    EXPECT_EQ(testNode.currentNeighbor()->i, 2);
    testNode.next();

    EXPECT_FALSE(testNode.isValid());

    EXPECT_EQ(testNode.getNode()->i, 0);
}

TEST(DeapthFirstSearch, exploreNeighborhood) {
    auto getNeighbors = [](std::vector<MoveOnly>& graph, MoveOnly const& node) -> std::vector<MoveOnly> {
        std::vector<MoveOnly> neighbors;
        if (graph.size() < static_cast<size_t>(node.i)) return neighbors;
        neighbors.emplace_back(node.i + 1);
        neighbors.emplace_back(node.i + 2);
        return neighbors;
    };

    auto deapthFirst = TestDeapthFirst<std::vector<MoveOnly>, MoveOnly, MoveOnly>{};
    auto graph = std::vector<MoveOnly>{};
    std::generate_n(std::back_inserter(graph), 10, []() {
        static int i = 0;
        return i++;
    });
    auto node = std::make_shared<MoveOnly>(0);
    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, node, getNeighbors));
    EXPECT_EQ(deapthFirst.getNeighbor()->i, 1);

    EXPECT_TRUE(deapthFirst.next());
    EXPECT_EQ(deapthFirst.getNeighbor()->i, 2);

    EXPECT_FALSE(deapthFirst.next());
}

TEST(DeapthFirstSearch, complex_explore_neihboor) {
    auto getNeighbors = [](std::vector<MoveOnly>& graph, MoveOnly const& node) -> std::vector<MoveOnly> {
        std::vector<MoveOnly> neighbors;
        if (graph.size() < static_cast<size_t>(node.i)) return neighbors;
        neighbors.emplace_back(node.i + 1);
        neighbors.emplace_back(node.i + 2);
        return neighbors;
    };

    auto deapthFirst = TestDeapthFirst<std::vector<MoveOnly>, MoveOnly, MoveOnly>{};
    auto graph = std::vector<MoveOnly>{};
    std::generate_n(std::back_inserter(graph), 10, []() {
        static int i = 0;
        return i++;
    });
    auto node = std::make_shared<MoveOnly>(0);
    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, node, getNeighbors));
    EXPECT_EQ(deapthFirst.getNeighbor()->i, 1);

    EXPECT_TRUE(deapthFirst.next());
    *node = 4;
    auto neighbor = deapthFirst.getNeighbor();
    EXPECT_EQ(neighbor->i, 2);

    EXPECT_TRUE(deapthFirst.exploreNeighborhood(graph, node, getNeighbors));
    EXPECT_EQ(deapthFirst.getNeighbor()->i, 5);

    EXPECT_EQ(neighbor->i, 2);
}
