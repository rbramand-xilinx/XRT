// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Devices, Inc. All rights reserved.

#include "graph.h"

namespace xrt::core::hip {
void
graph::
add_node(std::shared_ptr<node> node)
{
  m_nodes.push_back(node);
  node->set_parent(shared_from_this());
}

void
graph::
remove_node(std::shared_ptr<node> node)
{
  m_nodes.erase(std::remove(m_nodes.begin(), m_nodes.end(), node), m_nodes.end());
}

std::vector<std::shared_ptr<node>>
graph::
get_root_nodes() const
{
  // nodes with zero in_degree count
  std::vector<std::shared_ptr<node>> root_nodes;
  for (const auto& node : m_nodes) {
    if (node->get_indegree_count() == 0)
      root_nodes.push_back(node);
  }
  return root_nodes;
}

std::vector<std::shared_ptr<node>>
graph::
get_leaf_nodes() const
{
  // nodes with zero out_degree count
  std::vector<std::shared_ptr<node>> leaf_nodes;
  for (const auto& node : m_nodes) {
    if (node->get_outdegree_count() == 0)
      leaf_nodes.push_back(node);
  }
  return leaf_nodes;
}

// global map for storing graph handles
// we should override clang-tidy warning by adding NOLINT since graph_cache is non-const parameter
xrt_core::handle_map<graph_handle, std::shared_ptr<graph>> graph_cache; //NOLINT
}
