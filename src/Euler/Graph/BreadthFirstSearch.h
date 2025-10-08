#pragma once
#include <algorithm>
#include <cassert>
#include <deque>
#include <utility>
#include <vector>

namespace Euler {
namespace Graph {
template<typename Graph, typename Node, typename Neighbor>
class BreadthFirstSearch {
public:
    virtual ~BreadthFirstSearch() = default;
    /*
     * getNeighbors(Graph, Node) -> std::vector<Neighbor>
     * isSolved(Graph, Node) -> boolean
     * MoveForward(Graph, Node, Neighbor) -> boolean
     *
     * Differences to DepthFirstSearch:
     * - No MoveBackward is required because BFS does not backtrack.
     * - The internal queue is kept unique: each node is visited exactly once.
     */
    template<typename GetNeighbors, typename IsSolved, typename Visitor, typename MoveForward>
    auto start(
        Graph& graph,
        Node& start,
        Visitor& visitor,
        GetNeighbors&& getNeighbors,
        IsSolved&& isSolved,
        MoveForward&& moveForward) -> bool {
        clear();
        visitor.init(graph, start);

        // mark start visited and enqueue
        visited.push_back(start);
        queue.push_back(start);

        // process queue
        bool result = mainLoop(
            graph,
            visitor,
            std::forward<GetNeighbors>(getNeighbors),
            std::forward<IsSolved>(isSolved),
            std::forward<MoveForward>(moveForward));

        visitor.shutdown();
        return result;
    }

    template<typename GetNeighbors, typename IsSolved, typename Visitor, typename MoveForward>
    auto goOn(
        Graph& graph, Visitor& visitor, GetNeighbors&& getNeighbors, IsSolved&& isSolved, MoveForward&& moveForward)
        -> bool {
        visitor.goOn(graph);
        auto result = this->mainLoop(
            graph,
            visitor,
            std::forward<GetNeighbors>(getNeighbors),
            std::forward<IsSolved>(isSolved),
            std::forward<MoveForward>(moveForward));
        visitor.shutdown();
        return result;
    }

    virtual auto buildPath() -> std::vector<Node> {
        // For BFS return the list of visited nodes in visitation order.
        auto result = visited;
        if (this->currentNode) {
            if (std::find(result.begin(), result.end(), *this->currentNode) == result.end()) {
                result.push_back(*this->currentNode);
            }
        }
        return result;
    }

protected:
    template<typename GetNeighbors, typename IsSolved, typename Visitor, typename MoveForward>
    auto mainLoop(
        Graph& graph, Visitor& visitor, GetNeighbors&& getNeighbors, IsSolved&& isSolved, MoveForward&& moveForward)
        -> bool {

        while (!queue.empty()) {
            auto current = queue.front();
            queue.pop_front();

            // keep a copy in the member so callbacks that expect a reference can use it
            this->currentNodeValue = current;
            this->currentNode = std::addressof(this->currentNodeValue);

            // if the current node already solves the problem, we are done
            if (isSolved(graph, *this->currentNode)) return true;

            auto neighbors = getNeighbors(graph, *this->currentNode);
            if (neighbors.empty()) continue;

            for (auto& neighbor : neighbors) {
                // beforeMove/afterMove operate on a temporary node copy so moveForward
                // implementations can modify the node (as with DFS tests) without
                // affecting the queued state.
                Node temp = *this->currentNode;
                visitor.beforeMove(graph, *this->currentNode, neighbor);
                auto const moveOk = moveForward(graph, temp, neighbor);
                visitor.afterMove(graph, temp, neighbor);

                if (isSolved(graph, temp)) {
                    // set currentNode to the solved node for clients that inspect it
                    this->currentNodeValue = std::move(temp);
                    this->currentNode = std::addressof(this->currentNodeValue);
                    return true;
                }

                if (!moveOk || !visitor.isSolveable()) {
                    continue;
                }

                // ensure uniqueness: enqueue only if not already visited
                if (std::find(visited.begin(), visited.end(), temp) == visited.end()) {
                    visited.push_back(temp);
                    queue.push_back(temp);
                }
            }
        }

        return false;
    }

    void clear() {
        queue.clear();
        visited.clear();
        currentNode = nullptr;
        currentNodeValue = Node{};
    }

    std::deque<Node> queue;
    std::vector<Node> visited;
    Node* currentNode{nullptr};
    Node currentNodeValue{};
};
} // namespace Graph
} // namespace Euler
