#pragma once
#include <algorithm>
#include <set>
#include <vector>

namespace Euler {
namespace Set {
struct Powerset {

    template<typename T>
    auto operator()(std::set<T> const& input) -> std::vector<std::set<T>> {
        std::vector<bool> contain(input.size(), false);
        std::vector<std::set<T>> result;
        result.push_back(std::set<T>{});
        std::vector<T> access(input.begin(), input.end());
        // Empty subset
        for (size_t i = 0; i < input.size(); i++) {
            contain[i] = true;
            do {
                std::set<T> buffer;
                for (size_t j = 0; j < input.size(); ++j) {
                    if (contain[j]) buffer.insert(access[j]);
                }
                result.push_back(buffer);

            } while (std::prev_permutation(contain.begin(), contain.begin() + input.size()));
        }
        return result;
    }
};
} // namespace Set
} // namespace Euler
