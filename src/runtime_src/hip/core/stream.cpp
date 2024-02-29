// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Device, Inc. All rights reserved.

#include "stream.h"

namespace xrt::core::hip {
stream::
stream(std::shared_ptr<context> ctx, unsigned int flags, bool is_null)
  : m_ctx{std::move(ctx)}
  , m_flags{flags}
  , null{is_null}
{}

void
stream::
synchronize()
{
  if (flags & hipStreamNonBlocking)
    return; // non blocking stream

  // lock streams before accessing
  std::lock_guard<std::mutex> lock(streams.mutex);
  for (const auto& [handle, stream] : streams.stream_cache) {
    // check if the streams are in same context, stream is blocking
    // and stream is not current stream
    if (this->m_ctx == stream->m_ctx &&
      !(stream->flags() & hipStreamNonBlocking) && handle != this) {
      // non null streams wait on null stream only and
      // null stream waits on all blocking streams
      if (!null && !stream.is_null())
        continue;
      stream.await_completion();
    }
  }
}

void
stream::
await_completion()
{
  std::lock_guard<std::mutex> lk(m);
  while(!cmd_queue.empty()) {
    auto cmd = cmd_queue.front();
    cmd->wait();
    cmd->set_status(complete);
    cmd_queue.pop();
  }
}

void
stream::
record_top_event(command* event)
{
  std::lock_guard<std::mutex> lk(m);
  top_event = event;
}

// Global map of streams
stream_set streams;
}

