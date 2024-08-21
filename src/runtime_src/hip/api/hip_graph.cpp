// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Devices, Inc. All rights reserved.

#include "hip/core/common.h"
#include "hip/core/graph.h"

namespace xrt::core::hip {
static graph_handle
hip_graph_create()
{
  return nullptr;
}

static node_handle
hip_graph_add_empty_node(hipGraph_t /*graph*/, const hipGraphNode_t* /*dependencies*/,
                         size_t /*numDependencies*/)
{
  return nullptr;
}

static node_handle
hip_graph_add_kernel_node(hipGraph_t /*graph*/, const hipGraphNode_t* /*dependencies*/,
                          size_t /*numDependencies*/, const hipKernelNodeParams* /*pNodeParams*/)
{
  return nullptr;
}

static node_handle
hip_graph_add_memcpy_node(hipGraph_t /*graph*/, const hipGraphNode_t* /*dependencies*/,
                          size_t /*numDependencies*/, const HIP_MEMCPY3D* /*copyParams*/, hipCtx_t /*ctx*/)
{
  return nullptr;
}

static node_handle
hip_graph_add_memset_node(hipGraph_t /*graph*/, const hipGraphNode_t* /*dependencies*/,
                          size_t /*numDependencies*/, const hipMemsetParams* /*pMemsetParams*/)
{
  return nullptr;
}

static node_handle
hip_graph_add_event_record_node(hipGraph_t /*graph*/, const hipGraphNode_t* /*dependencies*/,
                                size_t /*numDependencies*/, hipEvent_t /*event*/)
{
  return nullptr;
}

template<typename Func, typename... Args>
static hipError_t
graph_add_node_helper(hipGraphNode_t* pGraphNode, hipGraph_t graph, const hipGraphNode_t* pDependencies,
                      size_t numDependencies, Func&& f, Args&&... args)
{
  try {
    throw_invalid_value_if(!pGraphNode, "Graph Node passed is nullptr");
    throw_invalid_value_if(!graph, "Graph passed is nullptr");
    throw_invalid_value_if(numDependencies > 0 && !pDependencies, "Pass proper values for dependencies");

    auto handle = f(graph, pDependencies, numDependencies, std::forward<Args>(args)...);
    *pGraphNode = reinterpret_cast<hipGraphNode_t>(handle);
    return hipSuccess;
  }
  catch (const xrt_core::system_error& ex) {
    xrt_core::send_exception_message(std::string(__func__) +  " - " + ex.what());
    return static_cast<hipError_t>(ex.value());
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
  }
  return hipErrorUnknown;
}
} // xrt::core::hip

// =========================================================================
// Graph related apis implementation
hipError_t
hipGraphCreate(hipGraph_t* pGraph, unsigned int flags)
{
  try {
    throw_invalid_value_if(!pGraph, "Graph passed is nullptr");
    throw_invalid_value_if(flags != 0, "flags should be 0");

    auto handle = xrt::core::hip::hip_graph_create();
    *pGraph = reinterpret_cast<hipGraph_t>(handle);
    return hipSuccess;
  }
  catch (const xrt_core::system_error& ex) {
    xrt_core::send_exception_message(std::string(__func__) +  " - " + ex.what());
    return static_cast<hipError_t>(ex.value());
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
  }
  return hipErrorUnknown;
}

hipError_t
hipGraphAddEmptyNode(hipGraphNode_t* pGraphNode, hipGraph_t graph,
                     const hipGraphNode_t* pDependencies, size_t numDependencies)
{
  return xrt::core::hip::graph_add_node_helper(pGraphNode, graph, pDependencies, numDependencies,
                                               xrt::core::hip::hip_graph_add_empty_node);
}

hipError_t
hipGraphAddKernelNode(hipGraphNode_t* pGraphNode, hipGraph_t graph, const hipGraphNode_t* pDependencies,
                      size_t numDependencies, const hipKernelNodeParams* pNodeParams)
{
  throw_invalid_value_if(!pNodeParams, "params passed is nullptr");
  return xrt::core::hip::graph_add_node_helper(pGraphNode, graph, pDependencies, numDependencies,
                                               xrt::core::hip::hip_graph_add_kernel_node, pNodeParams);
}

hipError_t
hipDrvGraphAddMemcpyNode(hipGraphNode_t* pGraphNode, hipGraph_t graph, const hipGraphNode_t* pDependencies,
                         size_t numDependencies, const HIP_MEMCPY3D* copyParams, hipCtx_t ctx)
{
  throw_invalid_value_if(!ctx, "context passed is nullptr");
  throw_invalid_value_if(!copyParams, "params passed is nullptr");
  return xrt::core::hip::graph_add_node_helper(pGraphNode, graph, pDependencies, numDependencies,
                                               xrt::core::hip::hip_graph_add_memcpy_node, copyParams, ctx);
}

hipError_t
hipGraphAddMemsetNode(hipGraphNode_t* pGraphNode, hipGraph_t graph, const hipGraphNode_t* pDependencies,
                      size_t numDependencies, const hipMemsetParams* pMemsetParams)
{
  throw_invalid_value_if(!pMemsetParams, "params passed is nullptr");
  return xrt::core::hip::graph_add_node_helper(pGraphNode, graph, pDependencies, numDependencies,
                                               xrt::core::hip::hip_graph_add_memset_node, pMemsetParams);
}

hipError_t
hipGraphAddEventRecordNode(hipGraphNode_t* pGraphNode, hipGraph_t graph, const hipGraphNode_t* pDependencies,
                           size_t numDependencies, hipEvent_t event)
{
  throw_invalid_value_if(!event, "Event passed is nullptr");
  return xrt::core::hip::graph_add_node_helper(pGraphNode, graph, pDependencies, numDependencies,
                                               xrt::core::hip::hip_graph_add_event_record_node, event);
}

hipError_t
hipGraphInstantiate(hipGraphExec_t* /*pGraphExec*/, hipGraph_t /*graph*/, hipGraphNode_t* /*pErrorNode*/,
                    char* /*pLogBuffer*/, size_t /*bufferSize*/)
{
  try {
    return hipSuccess;
  }
  catch (const xrt_core::system_error& ex) {
    xrt_core::send_exception_message(std::string(__func__) +  " - " + ex.what());
    return static_cast<hipError_t>(ex.value());
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
  }
  return hipErrorUnknown;
}

hipError_t
hipGraphLaunch(hipGraphExec_t /*graphExec*/, hipStream_t /*stream*/)
{
  try {
    return hipSuccess;
  }
  catch (const xrt_core::system_error& ex) {
    xrt_core::send_exception_message(std::string(__func__) +  " - " + ex.what());
    return static_cast<hipError_t>(ex.value());
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
  }
  return hipErrorUnknown;
}

hipError_t
hipGraphExecDestroy(hipGraphExec_t /*graphExec*/)
{
  try {
    return hipSuccess;
  }
  catch (const xrt_core::system_error& ex) {
    xrt_core::send_exception_message(std::string(__func__) +  " - " + ex.what());
    return static_cast<hipError_t>(ex.value());
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
  }
  return hipErrorUnknown;
}

hipError_t
hipGraphDestroy(hipGraph_t /*graph*/)
{
  try {
    return hipSuccess;
  }
  catch (const xrt_core::system_error& ex) {
    xrt_core::send_exception_message(std::string(__func__) +  " - " + ex.what());
    return static_cast<hipError_t>(ex.value());
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
  }
  return hipErrorUnknown;
}
