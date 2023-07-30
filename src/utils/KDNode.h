//
// Created by Louis on 24/09/2021.
//

#pragma once

#include <memory>
#include <vector>

template<typename T>
class KDNode {
private:
    std::shared_ptr<KDNode> m_left;
    std::shared_ptr<KDNode> m_right;

    size_t m_dim;
    std::vector<double> m_coords;

    T m_data;

    size_t m_axis;

    std::vector<double> m_subspace;

    bool m_is_right = true;

public:
    KDNode(std::vector<double> coords_, T data_, size_t axis_, bool is_right=true)
        : m_coords(std::move(coords_)), m_axis(axis_), m_data(data_), m_is_right(is_right)
    {
        m_dim = m_coords.size();
        m_left = nullptr;
        m_right = nullptr;

        for (int i = 0; i < m_dim*2; i++)
            m_subspace.emplace_back(std::numeric_limits<double>::infinity() * (i%2==0?-1:1));
    }

    inline std::shared_ptr<KDNode<T>> right() { return m_right; }
    inline std::shared_ptr<KDNode<T>> left() { return m_left; }

    inline void set_right(std::shared_ptr<KDNode<T>> node_)
    {
        node_->m_subspace = m_subspace;
        node_->m_subspace[m_axis*2] = m_coords[m_axis];
        node_->set_is_right(true);
        m_right = node_;
    }

    inline void set_left(std::shared_ptr<KDNode<T>> node_)
    {
        node_->m_subspace = m_subspace;
        node_->m_subspace[m_axis*2 + 1] = m_coords[m_axis];
        node_->set_is_right(false);
        m_left = node_;
    }

    inline void set_is_right(bool r) { m_is_right = r; }

    [[nodiscard]] inline bool is_right() const { return m_is_right; }

    [[nodiscard]] double coord(size_t axis_) const { return m_coords[axis_]; }

    [[nodiscard]] std::vector<double> coords() const { return m_coords; }

    [[nodiscard]] inline size_t dim() const noexcept { return m_dim; }

    [[nodiscard]] inline T data() const noexcept { return m_data; }

    [[nodiscard]] inline size_t axis() const noexcept { return m_axis; }

    [[nodiscard]] inline size_t next_axis() const noexcept
    {
        return (m_axis+1)%m_dim;
    }

    [[nodiscard]] double dist(const std::shared_ptr<KDNode<T>> other) const
    {
        return dist(other->m_coords);
    }

    [[nodiscard]] double dist(const std::vector<double>& coords_) const
    {
        double acc = 0;
        for (int i = 0; i < m_dim; i++) {
            auto d = m_coords[i] - coords_[i];
            acc += d*d;
        }
        acc = sqrt(acc);
        return acc;
    }

    [[nodiscard]] double dist_to_subspace(const std::vector<double>& subspace_) const
    {
        std::vector<double> dist(m_dim, 0);
        for (int i = 0; i < m_dim; i++) {
            if (m_coords[i] > subspace_[i*2] && m_coords[i] < subspace_[i*2 + 1])
                dist[i] = 0;
            else
                dist[i] = std::min(std::abs(subspace_[i*2] - m_coords[i]), std::abs(subspace_[i*2 + 1] - m_coords[i]));
        }
        double acc = 0;
        for (int i = 0; i < m_dim; i++) acc += dist[i]*dist[i];
        return sqrt(acc);
    }

    [[nodiscard]] double subspace_dist_to_point(const std::vector<double>& point) const
    {
        std::vector<double> dist(m_dim, 0);
        for (int i = 0; i < m_dim; i++) {
            if (point[i] > m_subspace[i*2] && point[i] < m_subspace[i*2 + 1])
                dist[i] = 0;
            else
                dist[i] = std::min(std::abs(m_subspace[i*2] - point[i]), std::abs(m_subspace[i*2 + 1] - point[i]));
        }
        double acc = 0;
        for (int i = 0; i < m_dim; i++) acc += dist[i]*dist[i];
        return sqrt(acc);
    }

    [[nodiscard]] bool in_subspace(const std::vector<double>& subspace_) const
    {
        for (int i = 0; i < m_dim; i++) {
            if (m_coords[i] < subspace_[i*2] || m_coords[i] > subspace_[i*2 + 1])
                return false;
        }
        return true;
    }
};
