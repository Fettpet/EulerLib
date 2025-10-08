#pragma once
#include <cassert>
#include <utility>
#include <vector>

namespace Euler {
namespace Graph {
template<typename Graph, typename Node, typename Neighbor>
class DepthFirstSearch {
public:
    virtual ~DepthFirstSearch() = default;
    /*
     * getNeighbors(Graph, Node) -> std::vector<Neighbor>
     * isSolved(Graph, Node) -> boolean
     * MoveForward(Graph, Node, Neighbor) -> boolean
     * MoveBackward(Graph, Node, Neighbor) -> void
     */
    template<typename GetNeighbors, typename IsSolved, typename Visitor, typename MoveForward, typename MoveBackward>
    auto start(
        Graph& graph,
        Node& start,
        Visitor& visitor,
        GetNeighbors&& getNeighbors,
        IsSolved&& isSolved,
        MoveForward&& moveForward,
        MoveBackward&& moveBackward) -> bool {
        clear();
        visitor.init(graph, start);

        this->currentNode = std::addressof(start);
        if (!this->exploreNeighborhood(graph, *this->currentNode, getNeighbors)) {
            return false;
        }

        auto result = mainLoop(
            graph,
            visitor,
            std::forward<GetNeighbors>(getNeighbors),
            std::forward<IsSolved>(isSolved),
            std::forward<MoveForward>(moveForward),
            std::forward<MoveBackward>(moveBackward));

        visitor.shutdown();
        return result;
    }

    template<typename GetNeighbors, typename IsSolved, typename Visitor, typename MoveForward, typename MoveBackward>
    auto goOn(
        Graph& graph,
        Visitor& visitor,
        GetNeighbors&& getNeighbors,
        IsSolved&& isSolved,
        MoveForward&& moveForward,
        MoveBackward&& moveBackward) -> bool {
        visitor.goOn(graph);
        auto& neighbor = this->getNeighbor();

        while (!this->pathStack.empty()) {
            // pass references to visitor/move callbacks
            visitor.beforeMoveBack(graph, *this->currentNode, neighbor);
            moveBackward(graph, *this->currentNode, neighbor);
            visitor.afterMoveBack(graph, *this->currentNode, neighbor);

            if (this->next()) {
                break;
            }
            else {
                this->pathStack.pop_back();
                if (this->pathStack.empty()) {
                    return isSolved(graph, *this->currentNode);
                }
            }
        }
        auto result = this->mainLoop(
            graph,
            visitor,
            std::forward<GetNeighbors>(getNeighbors),
            std::forward<IsSolved>(isSolved),
            std::forward<MoveForward>(moveForward),
            std::forward<MoveBackward>(moveBackward));
        visitor.shutdown();
        return result;
    }

    virtual auto buildPath() -> std::vector<Node> {
        auto result = std::vector<Node>{};
        for (auto& pathNode : pathStack) {
            result.push_back(std::move(pathNode).getNode());
        }
        if (currentNode) result.push_back(std::move(*currentNode));
        return result;
    }

    virtual auto getNeighborsPath() -> std::vector<Neighbor> {
        std::vector<Neighbor> result;
        for (auto& neighbor : pathStack) {
            result.push_back(std::move(neighbor).currentNeighbor());
        }
        return result;
    }

protected:
    template<typename GetNeighbors, typename IsSolved, typename Visitor, typename MoveForward, typename MoveBackward>
    auto mainLoop(
        Graph& graph,
        Visitor& visitor,
        GetNeighbors&& getNeighbors,
        IsSolved&& isSolved,
        MoveForward&& moveForward,
        MoveBackward&& moveBackward) -> bool {
        while (!this->pathStack.empty()) {
            auto& neighbor = this->getNeighbor();
            // pass references to visitor/move callbacks
            visitor.beforeMove(graph, *this->currentNode, neighbor);
            auto const solveable = moveForward(graph, *this->currentNode, neighbor);
            visitor.afterMove(graph, *this->currentNode, neighbor);
            if (isSolved(graph, *this->currentNode)) {
                return true;
            }
            if (!solveable || !visitor.isSolveable()) {
                this->gotoNextNeighbor(
                    graph, *this->currentNode, neighbor, visitor, std::forward<MoveBackward>(moveBackward));
                continue;
            }
            if (!this->exploreNeighborhood(graph, *this->currentNode, std::forward<GetNeighbors>(getNeighbors))) {
                this->gotoNextNeighbor(
                    graph, *this->currentNode, neighbor, visitor, std::forward<MoveBackward>(moveBackward));
                continue;
            }
        }
        return false;
    }

    template<typename Visitor, typename MoveBackward>
    auto gotoNextNeighbor(
        Graph& graph, Node& currentNode, Neighbor& neighbor, Visitor& visitor, MoveBackward&& moveBackward) -> bool {

        while (!this->pathStack.empty()) {
            // pass references to visitor/move callbacks
            visitor.beforeMoveBack(graph, currentNode, neighbor);
            moveBackward(graph, currentNode, neighbor);
            visitor.afterMoveBack(graph, currentNode, neighbor);

            if (this->next()) {
                return true;
            }
            this->pathStack.pop_back();
            if (this->pathStack.empty()) return false;
        }
        return false;
    }

    auto getNeighbor() -> Neighbor& {
        assert(!this->pathStack.empty());
        return this->getLastNeighbor().currentNeighbor();
    }

    template<typename GetNeighbors>
    auto exploreNeighborhood(Graph& graph, Node& node, GetNeighbors&& getNeighbors) -> bool {
        auto&& neighbors = getNeighbors(graph, node);
        if (neighbors.empty()) return false;
        this->pathStack.emplace_back(node, std::move(neighbors));
        return this->getLastNeighbor().isValid();
    }

    auto next() -> bool {
        if (this->pathStack.empty()) return false;
        if (!this->getLastNeighbor().isValid()) {
            return false;
        }
        this->getLastNeighbor().next();
        return this->getLastNeighbor().isValid();
    }

    void clear() {
        pathStack.clear();
        currentNode = nullptr;
    }

    struct PathNode {

        PathNode(Node&& startNode, std::vector<Neighbor>&& neighborList)
            : start(std::move(startNode))
            , neighbors(std::move(neighborList))
            , currentNeighborIndex(0) {}

        PathNode(Node const& startNode, std::vector<Neighbor>&& neighborList)
            : start(startNode)
            , neighbors(std::move(neighborList))
            , currentNeighborIndex(0) {}

        auto isValid() const -> bool { return currentNeighborIndex < neighbors.size(); }
        void next() { ++currentNeighborIndex; }

        auto currentNeighbor() & -> Neighbor& {
            assert(!neighbors.empty());
            assert(currentNeighborIndex < neighbors.size());
            return neighbors[currentNeighborIndex];
        }
        auto currentNeighbor() const& -> const Neighbor& {
            assert(!neighbors.empty());
            assert(currentNeighborIndex < neighbors.size());
            return neighbors[currentNeighborIndex];
        }

        auto currentNeighbor() && -> Neighbor { return std::move(neighbors[currentNeighborIndex]); }
        auto getNode() const& -> const Node& { return start; }
        auto getNode() && -> Node { return std::move(start); }

    protected:
        Node start;
        std::vector<Neighbor> neighbors;
        std::size_t currentNeighborIndex;
    };

    auto getLastNeighbor() -> PathNode& {
        assert(!pathStack.empty());
        return pathStack.back();
    }

    auto getLastNeighbor() const -> PathNode const& {
        assert(!pathStack.empty());
        return pathStack.back();
    }

    std::vector<PathNode> pathStack;
    Node* currentNode{nullptr};
};
} // namespace Graph
} // namespace Euler
