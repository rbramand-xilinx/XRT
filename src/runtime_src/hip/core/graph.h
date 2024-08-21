// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Devices, Inc. All rights reserved.
#ifndef xrthip_graph_h
#define xrthip_graph_h

#include "common.h"

#include <memory>
#include <vector>

namespace xrt::core::hip {

// graph_handle - opaque graph handle
using graph_handle = void*;
using node_handle = void*;

// forward declaration
class graph;

class node : public std::enable_shared_from_this<node>
{
  enum class node_type : uint16_t 
  {
    empty = 0,
    memcpy,
    memset,
    kernel,
    event
  };
  node_type m_type = node_type::empty;
  std::shared_ptr<graph> m_graph;
  std::vector<std::shared_ptr<node>> m_edges;
  std::vector<std::shared_ptr<node>> m_dependencies;
  // count of incoming and outgoing edges
  uint32_t m_in_degree = 0;
  uint32_t m_out_degree = 0;

public:
  explicit
  node(node_type type)
    : m_type{type}
  {}

  void
  set_parent(std::shared_ptr<graph> graph)
  {
    m_graph = graph;
  }

  void
  add_edge(std::shared_ptr<node> edge)
  {
    m_edges.push_back(edge);
    ++m_out_degree;
  }

  void
  add_dependency(std::shared_ptr<node> dependency)
  {
    m_dependencies.push_back(dependency);
    dependency->add_edge(shared_from_this());
    ++m_in_degree;
  }

  const std::vector<std::shared_ptr<node>>&
  get_edges() const { return m_edges; }

  uint32_t
  get_indegree_count() const { return m_in_degree; }

  uint32_t
  get_outdegree_count() const { return m_out_degree; }
};

class graph : public std::enable_shared_from_this<graph>
{
  std::vector<std::shared_ptr<node>> m_nodes;
  bool m_instantiated = false;
  std::shared_ptr<context> m_ctx;

public:
  explicit
  graph(std::shared_ptr<context> ctx)
    : m_ctx{ctx}
  {} 

  void
  add_node(std::shared_ptr<node> node);

  void
  remove_node(std::shared_ptr<node> node);

  std::vector<std::shared_ptr<node>>
  get_root_nodes() const;

  std::vector<std::shared_ptr<node>>
  get_leaf_nodes() const;

  bool
  is_instantiated() const { return m_instantiated; }

  size_t
  get_node_count() const { return m_nodes.size(); }

  const std::vector<std::shared_ptr<node>>&
  get_nodes() const { return m_nodes; }
};

// Global map of graphs
extern xrt_core::handle_map<graph_handle, std::shared_ptr<graph>> graph_cache;
} // xrt::core::hip

#endif
