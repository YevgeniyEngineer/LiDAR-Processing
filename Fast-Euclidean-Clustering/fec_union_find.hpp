#ifndef UNION_FIND_HPP
#define UNION_FIND_HPP

#include <cstdint>
#include <numeric>
#include <type_traits>
#include <vector>

namespace clustering
{
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>> class FECUnionFind
{
  public:
    FECUnionFind() = delete;
    FECUnionFind(const std::size_t size) : parent_(size), rank_(size, 0)
    {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    T find(const T x)
    {
        auto &parent_x = parent_[x];
        if (parent_x != x)
        {
            parent_x = find(parent_x);
        }
        return parent_x;
    }

    void merge(const T x, const T y)
    {
        const T root_x = find(x);
        const T root_y = find(y);

        if (root_x == root_y)
        {
            return;
        }

        if (rank_[root_x] < rank_[root_y])
        {
            parent_[root_x] = root_y;
        }
        else if (rank_[root_x] > rank_[root_y])
        {
            parent_[root_y] = root_x;
        }
        else
        {
            parent_[root_y] = root_x;
            ++rank_[root_x];
        }
    }

  private:
    std::vector<T> parent_;
    std::vector<T> rank_;
};

} // namespace clustering

#endif // UNION_FIND_HPP