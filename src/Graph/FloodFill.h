#pragma once
#include <set>
#include <stack>
#include <utility>
#include <vector>

namespace Algorithm {

template<typename Graph, typename Node, typename IsPart, typename Neighbors, typename Compare = std::less<Node>>
class FloodFill : private IsPart, private Neighbors {

public:
    FloodFill(Graph const& newGraph) { graph = &newGraph; }

    [[nodiscard]] inline auto calculate(Node const& start) -> std::vector<Node> {
        if (isInvalidStartNode(start)) {
            return std::vector<Node>{};
        }
        initializeCalculationObjects(start);

        while (!toLook.empty()) {
            auto currentNode = getNextNode();
            members.insert(currentNode);
            exploreNeighborhood(currentNode);
        }
        return std::vector<Node>(members.begin(), members.end());
    }

    void clearVisited() { visited.clear(); }

    void setVisited(std::set<Node> const& vi) { visited = vi; }

    std::set<Node>& getVisitedRef() { return visited; }

private:
    void exploreNeighborhood(Node const& node) {
        auto neighbors = getNeighbors(*graph, node);
        for (auto const& neighbor : neighbors) {
            if (isPart(*graph, neighbor)) {
                appendNodeToStack(neighbor);
            }
        }
    }

    [[nodiscard]] auto getNextNode() -> Node {
        auto currentNode = toLook.top();
        toLook.pop();
        return currentNode;
    }

    void appendNodeToStack(Node const& node) {
        if (!isVisited(node)) {
            toLook.push(node);
        }
        visited.insert(node);
    }

    void initializeCalculationObjects(Node const& node) {
        toLook = std::stack<Node>{};
        toLook.push(node);
        members = {node};
    }

    [[nodiscard]] auto isVisited(Node const& node) const -> bool { return visited.count(node) || members.count(node); }

    [[nodiscard]] auto isInvalidStartNode(Node const& start) const -> bool {
        return !isPart(*graph, start) || visited.count(start);
    }

protected:
    using IsPart::isPart;
    using Neighbors::getNeighbors;

protected:
    Graph const* graph = nullptr;
    std::set<Node, Compare> visited{Compare{}};
    std::stack<Node> toLook;
    std::set<Node, Compare> members{Compare{}};
};

} // namespace Algorithm
