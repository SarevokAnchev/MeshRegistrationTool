//
// Created by Louis on 24/09/2021.
//

#pragma once

#include <memory>
#include <vector>
#include <algorithm>
#include <random>

#include "KDNode.h"

template <typename T>
class KDTree {
private:
    std::shared_ptr<KDNode<T>> m_root;

    size_t m_dim;

    void _explore_from_top(const std::vector<double>& coords, std::shared_ptr<KDNode<T>> node,
                           std::shared_ptr<KDNode<T>>& best_node, double* best_dist) const
    {
        if (node->subspace_dist_to_point(coords) > *best_dist)
            return;
        auto cur_dist = node->dist(coords);
        if (cur_dist < *best_dist) {
            *best_dist = cur_dist;
            best_node = node;
        }
        if (node->right() && node->right()->subspace_dist_to_point(coords) < *best_dist) {
            _explore_from_top(coords, node->right(), best_node, best_dist);
        }
        if (node->left() && node->left()->subspace_dist_to_point(coords) < *best_dist) {
            _explore_from_top(coords, node->left(), best_node, best_dist);
        }
    }

    void _explore_with_range(const std::vector<double>& coords, std::shared_ptr<KDNode<T>> node,
                             double range, std::vector<std::shared_ptr<KDNode<T>>>& found) const
    {
        if (node->subspace_dist_to_point(coords) > range)
            return;
        auto cur_dist = node->dist(coords);
        if (cur_dist < range) {
            found.emplace_back(node);
        }
        if (node->right() && node->right()->subspace_dist_to_point(coords) < range) {
            _explore_with_range(coords, node->right(), range, found);
        }
        if (node->left() && node->left()->subspace_dist_to_point(coords) < range) {
            _explore_with_range(coords, node->left(), range, found);
        }
    }

public:
    KDTree()
            : m_root(nullptr), m_dim(0)
    {}

    void add_nodes(const std::vector<std::vector<double>>& coords_, const std::vector<T>& data_, bool shuffle=false)
    {
        if (shuffle) {
            std::vector<size_t> indices(coords_.size());
            for (auto i = 0; i < coords_.size(); i++) indices[i] = i;
            std::shuffle(indices.begin(), indices.end(), std::default_random_engine{});
            for (auto i: indices) {
                add_node(coords_[i], data_[i]);
            }
        }
        else {
            for (int i = 0; i < coords_.size(); i++) {
                add_node(coords_[i], data_[i]);
            }
        }
    }

    void add_node(const std::vector<double>& coords_, const T& data_)
    {
        if (!m_root) {
            m_root = std::make_shared<KDNode<T>>(coords_, data_, 0);
            m_dim = coords_.size();
            return;
        }
        bool placed = false;
        std::shared_ptr<KDNode<T>> ptr_next = m_root;
        std::shared_ptr<KDNode<T>> ptr_prev;
        while (!placed) {
            auto axis = ptr_next->axis();
            ptr_prev = ptr_next;
            if (ptr_next->coord(axis) > coords_[axis]) {
                ptr_next = ptr_next->left();
                if (!ptr_next) {
                    ptr_prev->set_left(std::make_shared<KDNode<T>>(coords_, std::move(data_), ptr_prev->next_axis()));
                    placed = true;
                }
            }
            else {
                ptr_next = ptr_next->right();
                if (!ptr_next) {
                    ptr_prev->set_right(std::make_shared<KDNode<T>>(coords_, std::move(data_), ptr_prev->next_axis()));
                    placed = true;
                }
            }
        }
    }

    std::shared_ptr<KDNode<T>> get_closest_node(const std::vector<double>& coords_) const
    {
        if (!m_root) {
            return nullptr;
        }
        // descend in the tree
        std::vector<std::shared_ptr<KDNode<T>>> path;
        bool last_branch_right = true;
        auto ptr_next = m_root;
        while (ptr_next) {
            auto axis = ptr_next->axis();
            path.emplace_back(ptr_next);
            if (ptr_next->coord(axis) > coords_[axis]) {
                ptr_next = ptr_next->left();
                last_branch_right = false;
            }
            else {
                ptr_next = ptr_next->right();
                last_branch_right = true;
            }
        }

        // ascend and compare distances
        int idx = path.size() - 1;
        std::shared_ptr<KDNode<T>> cur_best = path.back();
        auto min_dist = cur_best->dist(coords_);
        while (idx >= 0) {
            auto d = path[idx]->dist(coords_);
            if (d < min_dist) {
                min_dist = d;
                cur_best = path[idx];
            }
            if (last_branch_right && path[idx]->left()) {
                _explore_from_top(coords_, path[idx]->left(), cur_best, &min_dist);
            }
            else if (path[idx]->right()){
                _explore_from_top(coords_, path[idx]->right(), cur_best, &min_dist);
            }
            last_branch_right = path[idx]->is_right();
            idx--;
        }
        return cur_best;
    }

    std::vector<std::shared_ptr<KDNode<T>>> range_search(const std::vector<double>& coords_, double range) const
    {
        if (!m_root) {
            return {nullptr};
        }
        std::vector<std::shared_ptr<KDNode<T>>> found;
        _explore_with_range(coords_, m_root, range, found);
        return found;
    }
};
