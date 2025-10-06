#pragma once
#include <cassert>
#include <memory>
#include <utility>
#include <vector>

namespace Euler {
namespace Graph {
template<typename Graph, typename Node, typename Neighbor>
class DeapthFirstSearch {
public:
    /*
     * getNeighbors(Graph, Node) -> std::vector<Neighbor>
     * isSolved(Graph, Node) -> boolean
     * MoveForward(Graph, Node, Neighbor) -> Node
     * MoveBackward(Graph, Node, Neighbor) -> Node
     */
    template<typename GetNeighbors, typename IsSolved, typename Visitor, typename MoveForward, typename MoveBackward>
    inline auto start(
        Graph& graph,
        Node& start,
        Visitor& visitor,
        GetNeighbors&& getNeighbors,
        IsSolved&& isSolved,
        MoveForward&& moveForward,
        MoveBackward&& moveBackward) -> bool {
        clear();
        visitor.init(graph, start);

        currentNode = std::make_shared<Node>(start);
        if (!exploreNeighborhood(graph, currentNode, getNeighbors)) {
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
    inline auto goOn(
        Graph& graph,
        Visitor& visitor,
        GetNeighbors&& getNeighbors,
        IsSolved&& isSolved,
        MoveForward&& moveForward,
        MoveBackward&& moveBackward) -> bool {
        visitor.goOn(graph);
        auto neighbor = getNeighbor();

        while (!neighborsList.empty()) {
            // pass references to visitor/move callbacks
            visitor.beforeMoveBack(graph, *currentNode, *neighbor);
            moveBackward(graph, *currentNode, *neighbor);
            visitor.afterMoveBack(graph, *currentNode, *neighbor);
            neighbor = getNeighbor();
            if (next()) {
                break;
            }
            else {
                neighborsList.pop_back();
                if (neighborsList.empty()) {
                    return isSolved(graph, *currentNode);
                }
                neighbor = getNeighbor();
            }
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

    virtual inline auto buildPath() const -> std::vector<std::shared_ptr<Node>> {
        auto result = std::vector<std::shared_ptr<Node>>{};
        for (auto const& pathNode : neighborsList) {
            result.push_back(pathNode->getNode());
        }
        result.push_back(currentNode);
        return result;
    }

    virtual inline auto getNeighborsPath() const -> std::vector<std::shared_ptr<Neighbor>> {
        std::vector<std::shared_ptr<Neighbor>> result;
        for (auto& neighbor : neighborsList) {
            result.push_back(neighbor->currentNeighbor());
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
        while (!neighborsList.empty()) {
            auto neighbor = getNeighbor();
            // pass references to visitor/move callbacks
            visitor.beforeMove(graph, *currentNode, *neighbor);
            auto const solveable = moveForward(graph, *currentNode, *neighbor);
            visitor.afterMove(graph, *currentNode, *neighbor);
            if (isSolved(graph, *currentNode)) {
                return true;
            }
            if (!solveable || !visitor.isSolveable()) {
                gotoNextNeighbor(graph, currentNode, neighbor, visitor, std::forward<MoveBackward>(moveBackward));
                continue;
            }
            if (!exploreNeighborhood(graph, currentNode, std::forward<GetNeighbors>(getNeighbors))) {
                gotoNextNeighbor(graph, currentNode, neighbor, visitor, std::forward<MoveBackward>(moveBackward));
                continue;
            }
        }
        return false;
    }

    template<typename Visitor, typename MoveBackward>
    inline auto gotoNextNeighbor(
        Graph& graph,
        std::shared_ptr<Node>& currentNode,
        std::shared_ptr<Neighbor>& neighbor,
        Visitor& visitor,
        MoveBackward&& moveBackward) -> bool {

        while (!neighborsList.empty()) {
            // pass references to visitor/move callbacks
            visitor.beforeMoveBack(graph, *currentNode, *neighbor);
            moveBackward(graph, *currentNode, *neighbor);
            visitor.afterMoveBack(graph, *currentNode, *neighbor);

            if (next()) {
                return true;
            }
            neighborsList.pop_back();
            if (neighborsList.empty()) return false;
            neighbor = getNeighbor();
        }
        return false;
    }

    inline auto getNeighbor() -> std::shared_ptr<Neighbor> {
        assert(!neighborsList.empty());
        return getLastNeighbor()->currentNeighbor();
    }

    template<typename GetNeighbors>
    inline auto exploreNeighborhood(Graph& graph, std::shared_ptr<Node> const& node, GetNeighbors&& getNeighbors)
        -> bool {
        auto&& neighbors = getNeighbors(graph, *node);
        if (neighbors.size() == 0) return false;
        neighborsList.emplace_back(std::make_unique<PathNode>(node, std::move(neighbors)));
        return getLastNeighbor()->isValid();
    }

    inline auto next() -> bool {
        if (neighborsList.empty()) return false;
        if (!getLastNeighbor()->isValid()) {
            return false;
        }
        getLastNeighbor()->next();
        return getLastNeighbor()->isValid();
    }

    inline void clear() { neighborsList.clear(); }

    struct PathNode {

        inline PathNode(std::shared_ptr<Node> const& start, std::vector<Neighbor>&& neighborList)
            : start(start) {
            if (neighborList.empty()) return;
            for (auto& neighbor : neighborList) {
                neighbors.emplace_back(std::make_shared<Neighbor>(std::move(neighbor)));
            }
            currentNeighborIt = neighbors.begin();
        }

        inline auto isValid() const -> bool { return currentNeighborIt != neighbors.end(); }
        inline void next() { ++currentNeighborIt; }

        inline auto currentNeighbor() const -> std::shared_ptr<Neighbor> {
            assert(!neighbors.empty());
            assert(currentNeighborIt != neighbors.end());
            return *currentNeighborIt;
        }

        auto getNode() const -> std::shared_ptr<Node> const& { return start; }

    protected:
        using NeighborIterator = typename std::vector<std::shared_ptr<Neighbor>>::iterator;
        std::shared_ptr<Node> start;
        std::vector<std::shared_ptr<Neighbor>> neighbors;
        NeighborIterator currentNeighborIt;
    };

    inline auto getLastNeighbor() -> std::unique_ptr<PathNode>& {
        assert(!neighborsList.empty());
        return neighborsList.back();
    }

    inline auto getLastNeighbor() const -> std::unique_ptr<PathNode> const& {
        assert(!neighborsList.empty());
        return neighborsList.back();
    }

    std::vector<std::unique_ptr<PathNode>> neighborsList;
    std::shared_ptr<Node> currentNode;
};
} // namespace Graph
} // namespace Euler
