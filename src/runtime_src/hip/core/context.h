// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Device, Inc. All rights reserved.
#ifndef xrthip_context_h
#define xrthip_context_h

#include "device.h"

namespace xrt::core::hip {

// context_handle - opaque context handle
using context_handle = void*;

// forward declaration
class stream;

class context : public std::enable_shared_from_this<context>
{
  std::shared_ptr<device> m_device;
  std::weak_ptr<stream> null_stream;

public:
  context() = default;

  context(std::shared_ptr<device> device);

  uint32_t
  get_dev_id() const
  {
    return m_device->get_device_id();
  }

  const xrt::device&
  get_xrt_device() const
  {
    return m_device->get_xrt_device();
  }

  std::shared_ptr<stream>
  get_null_stream();
};

std::shared_ptr<context>
get_current_context();

context_handle
hip_device_primary_ctx_retain(device_handle dev);

extern xrt_core::handle_map<context_handle, std::shared_ptr<context>> context_cache;
} // xrt::core::hip

#endif

