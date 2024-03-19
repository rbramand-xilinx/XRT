// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2023 Advanced Micro Device, Inc. All rights reserved.
#ifndef xrthip_memory_h
#define xrthip_memory_h

#include "xrt/xrt_bo.h"


namespace xrt::core::hip {
using mem_handle = void*;

class memory
{
  xrt::bo m_xrt_bo;

public:
  memory(xrt::bo xrt_bo)
    : m_xrt_bo(xrt_bo)
  {}

  xrt::bo&
  get_xrt_bo()
  {
    return m_xrt_bo;
  }
};

extern xrt_core::handle_map<mem_handle, std::shared_ptr<memory>> mem_cache;
  
} // xrt::core::hip

#endif
