// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Device, Inc. All rights reserved.
#ifndef xrthip_stream_h
#define xrthip_stream_h

#include "context.h"

#include <condition_variable>
#include <queue>

namespace xrt::core::hip {

// stream_handle - opaque stream handle
using stream_handle = void*;

// forward declaration
class command;
class stream
{
  std::shared_ptr<context> m_ctx;
  unsigned int m_flags;
  bool null;

  std::queue<std::shared_ptr<command>> cmd_queue;
  std::mutex m;
  std::condition_variable cv;
  command* top_event{nullptr};

public:
  stream() = default;
  stream(std::shared_ptr<context> ctx, unsigned int flags, bool is_null = false);

  inline bool
  is_null() const
  {
    return null;
  }

  unsigned int
  flags() const
  {
    return m_flags;
  }

  void
  synchronize();

  void
  await_completion();

  void
  record_top_event(command* event);
};

struct stream_set
{
  std::mutex mutex;
  xrt_core::handle_map<stream_handle, std::shared_ptr<stream>> stream_cache;
};
extern stream_set streams;
} // xrt::core::hip

#endif

