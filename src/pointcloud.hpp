#include <cassert>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_processing
{
template <typename PointT> class PointCloud final
{
  public:
    class PointsView final
    {
        using IteratorT = typename std::vector<PointT>::iterator;
        using ConstIteratorT = typename std::vector<PointT>::const_iterator;
        using ReverseIteratorT = typename std::vector<PointT>::reverse_iterator;
        using ConstReverseIteratorT = typename std::vector<PointT>::const_reverse_iterator;

      public:
        explicit PointsView(PointCloud<PointT> *parent, std::size_t cluster_index);

        IteratorT begin() const noexcept;
        IteratorT end() const noexcept;
        ConstIteratorT cbegin() const noexcept;
        ConstIteratorT cend() const noexcept;
        ReverseIteratorT rbegin() const noexcept;
        ReverseIteratorT rend() const noexcept;
        ConstReverseIteratorT crbegin() const noexcept;
        ConstReverseIteratorT crend() const noexcept;

        std::size_t size() const noexcept;
        PointT &operator[](std::size_t index) noexcept;
        const PointT &operator[](std::size_t index) const noexcept;
        PointT &at(std::size_t index);
        const PointT &at(std::size_t index) const;
        PointT &at_point(std::size_t index);
        const PointT &at_point(std::size_t index) const;

        void push_back(const PointT &point);
        void push_back(PointT &&point);
        template <typename... Args> void emplace_back(Args &&...args);

      private:
        PointCloud<PointT> *parent_{nullptr};
        std::size_t cluster_index_{std::size_t(-1)};
    };

    PointCloud() = default;
    PointCloud(std::size_t initial_capacity);
    ~PointCloud() = default;

    PointCloud(const PointCloud &) = default;
    PointCloud &operator=(const PointCloud &) = default;
    PointCloud(PointCloud &&) = default;
    PointCloud &operator=(PointCloud &&) = default;

    void reserve_max_points(std::size_t capacity);
    template <typename... Args> void emplace_into_last_cluster(Args &&...args);

    void push_into_last_cluster(const PointT &point);
    void push_into_last_cluster(PointT &&point);
    void increment_cluster();
    void clear() noexcept;

    std::size_t total_clusters() const noexcept;
    std::size_t total_points() const noexcept;

    PointT &operator()(std::size_t cluster_index, std::size_t point_index);
    const PointT &operator()(std::size_t cluster_index, std::size_t point_index) const;
    PointT &at(std::size_t cluster_index, std::size_t point_index);
    const PointT &at(std::size_t cluster_index, std::size_t point_index) const;

    void add_cluster(const std::vector<PointT> &new_cluster);

    const PointsView at_cluster(std::size_t cluster_index) const;

    PointsView last_cluster();
    const PointsView last_cluster() const;

    void pop_last_cluster() noexcept;
    void print_clusters() const;

  private:
    std::vector<PointT> points_;                                // Main buffer for all points
    std::vector<std::pair<std::size_t, std::size_t>> clusters_; // Start index and size for each cluster

    void ensure_capacity(std::size_t required_capacity);
};

template <typename PointT>
PointCloud<PointT>::PointsView::PointsView(PointCloud<PointT> *parent, std::size_t cluster_index)
    : parent_(parent), cluster_index_(cluster_index)
{
}

template <typename PointT>
typename PointCloud<PointT>::PointsView::IteratorT PointCloud<PointT>::PointsView::begin() const noexcept
{
    return parent_->points_.begin() + parent_->clusters_[cluster_index_].first;
}

template <typename PointT>
typename PointCloud<PointT>::PointsView::IteratorT PointCloud<PointT>::PointsView::end() const noexcept
{
    return parent_->points_.begin() + parent_->clusters_[cluster_index_].first +
           parent_->clusters_[cluster_index_].second;
}

template <typename PointT>
typename PointCloud<PointT>::PointsView::ConstIteratorT PointCloud<PointT>::PointsView::cbegin() const noexcept
{
    return parent_->points_.cbegin() + parent_->clusters_[cluster_index_].first;
}

template <typename PointT>
typename PointCloud<PointT>::PointsView::ConstIteratorT PointCloud<PointT>::PointsView::cend() const noexcept
{
    return parent_->points_.cbegin() + parent_->clusters_[cluster_index_].first +
           parent_->clusters_[cluster_index_].second;
}

template <typename PointT>
typename PointCloud<PointT>::PointsView::ReverseIteratorT PointCloud<PointT>::PointsView::rbegin() const noexcept
{
    return ReverseIteratorT(end());
}

template <typename PointT>
typename PointCloud<PointT>::PointsView::ReverseIteratorT PointCloud<PointT>::PointsView::rend() const noexcept
{
    return ReverseIteratorT(begin());
}

template <typename PointT>
typename PointCloud<PointT>::PointsView::ConstReverseIteratorT PointCloud<PointT>::PointsView::crbegin() const noexcept
{
    return ConstReverseIteratorT(cend());
}

template <typename PointT>
typename PointCloud<PointT>::PointsView::ConstReverseIteratorT PointCloud<PointT>::PointsView::crend() const noexcept
{
    return ConstReverseIteratorT(cbegin());
}

template <typename PointT> std::size_t PointCloud<PointT>::PointsView::size() const noexcept
{
    return parent_->clusters_[cluster_index_].second;
}

template <typename PointT> PointT &PointCloud<PointT>::PointsView::operator[](std::size_t index) noexcept
{
    assert(index < size());
    return parent_->points_[parent_->clusters_[cluster_index_].first + index];
}

template <typename PointT> const PointT &PointCloud<PointT>::PointsView::operator[](std::size_t index) const noexcept
{
    assert(index < size());
    return parent_->points_[parent_->clusters_[cluster_index_].first + index];
}

template <typename PointT> PointT &PointCloud<PointT>::PointsView::at(std::size_t index)
{
    if (index >= size())
    {
        throw std::runtime_error("PointCloud::PointsView::at(): Index out of bounds");
    }
    return parent_->points_[parent_->clusters_[cluster_index_].first + index];
}

template <typename PointT> const PointT &PointCloud<PointT>::PointsView::at(std::size_t index) const
{
    if (index >= size())
    {
        throw std::runtime_error("PointCloud::PointsView::at(): Index out of bounds");
    }
    return parent_->points_[parent_->clusters_[cluster_index_].first + index];
}

template <typename PointT> PointT &PointCloud<PointT>::PointsView::at_point(std::size_t index)
{
    if (index >= size())
    {
        throw std::runtime_error("PointCloud::PointsView::at_point(): Index out of bounds");
    }
    return parent_->points_[parent_->clusters_[cluster_index_].first + index];
}

template <typename PointT> const PointT &PointCloud<PointT>::PointsView::at_point(std::size_t index) const
{
    if (index >= size())
    {
        throw std::runtime_error("PointCloud::PointsView::at_point(): Index out of bounds");
    }
    return parent_->points_[parent_->clusters_[cluster_index_].first + index];
}

template <typename PointT> void PointCloud<PointT>::PointsView::push_back(const PointT &point)
{
    parent_->push_into_last_cluster(point);
}

template <typename PointT> void PointCloud<PointT>::PointsView::push_back(PointT &&point)
{
    parent_->push_into_last_cluster(std::move(point));
}

template <typename PointT> template <typename... Args> void PointCloud<PointT>::PointsView::emplace_back(Args &&...args)
{
    parent_->emplace_into_last_cluster(std::forward<Args>(args)...);
}

template <typename PointT> PointCloud<PointT>::PointCloud(std::size_t initial_capacity)
{
    points_.reserve(initial_capacity);
    clusters_.reserve(initial_capacity); // Assume 1 pt per cluster worst case scenario
}

template <typename PointT> void PointCloud<PointT>::reserve_max_points(std::size_t capacity)
{
    points_.reserve(capacity);
    clusters_.reserve(capacity); // Same assumption as above
}

template <typename PointT>
template <typename... Args>
void PointCloud<PointT>::emplace_into_last_cluster(Args &&...args)
{
    if (clusters_.empty())
    {
        clusters_.emplace_back(points_.size(), 0); // Start a new cluster if none exist
    }
    auto &last = clusters_.back();
    ensure_capacity(last.first + last.second + 1);
    points_.emplace_back(std::forward<Args>(args)...);
    ++last.second;
}

template <typename PointT> void PointCloud<PointT>::push_into_last_cluster(const PointT &point)
{
    emplace_into_last_cluster(point);
}

template <typename PointT> void PointCloud<PointT>::push_into_last_cluster(PointT &&point)
{
    emplace_into_last_cluster(std::move(point));
}

template <typename PointT> void PointCloud<PointT>::increment_cluster()
{
    if (!clusters_.empty())
    {
        auto &last_cluster = clusters_.back();
        ensure_capacity(last_cluster.first + last_cluster.second + 1);
    }
    clusters_.emplace_back(points_.size(), 0);
}

template <typename PointT> void PointCloud<PointT>::clear() noexcept
{
    points_.clear();
    clusters_.clear();
}

template <typename PointT> std::size_t PointCloud<PointT>::total_clusters() const noexcept
{
    return clusters_.size();
}

template <typename PointT> std::size_t PointCloud<PointT>::total_points() const noexcept
{
    return points_.size();
}

template <typename PointT> void PointCloud<PointT>::add_cluster(const std::vector<PointT> &new_cluster)
{
    const std::size_t start_index = points_.size();
    points_.insert(points_.end(), new_cluster.cbegin(), new_cluster.cend());
    clusters_.emplace_back(start_index, new_cluster.size());
}

template <typename PointT> typename PointCloud<PointT>::PointsView PointCloud<PointT>::last_cluster()
{
    if (clusters_.empty())
    {
        throw std::runtime_error("PointCloud::last_cluster(): No clusters");
    }
    return PointsView(this, clusters_.size() - 1);
}

template <typename PointT> const typename PointCloud<PointT>::PointsView PointCloud<PointT>::last_cluster() const
{
    if (clusters_.empty())
    {
        throw std::runtime_error("PointCloud::last_cluster(): No clusters");
    }
    return PointsView(const_cast<PointCloud<PointT> *>(this), clusters_.size() - 1);
}

template <typename PointT>
const typename PointCloud<PointT>::PointsView PointCloud<PointT>::at_cluster(std::size_t cluster_index) const
{
    if (cluster_index >= clusters_.size())
    {
        throw std::runtime_error("PointCloud::at_cluster(): Index out of bounds");
    }
    return PointsView(const_cast<PointCloud<PointT> *>(this), cluster_index);
}

template <typename PointT> void PointCloud<PointT>::pop_last_cluster() noexcept
{
    if (!clusters_.empty())
    {
        auto &last = clusters_.back();
        points_.resize(last.first);
        clusters_.pop_back();
    }
}

template <typename PointT> void PointCloud<PointT>::print_clusters() const
{
    for (const auto &cluster : clusters_)
    {
        std::cout << "Cluster starting at index " << cluster.first << " with size " << cluster.second << std::endl;
    }
}

template <typename PointT> void PointCloud<PointT>::ensure_capacity(std::size_t required_capacity)
{
    if (required_capacity > points_.capacity())
    {
        points_.reserve(required_capacity);
    }
}
}; // namespace lidar_processing
