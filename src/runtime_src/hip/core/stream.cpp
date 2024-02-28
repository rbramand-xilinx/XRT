// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Device, Inc. All rights reserved.

#include "stream.h"

namespace xrt::core::hip {
stream::
stream(std::shared_ptr<context> ctx, unsigned int flags)
  : m_ctx(std::move(ctx)),
  : m_flags(flags)
{}

// Global map of streams
xrt_core::handle_map<stream_handle, std::shared_ptr<stream>> stream_cache;
}

