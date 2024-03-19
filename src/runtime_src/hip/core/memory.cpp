// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2023 Advanced Micro Device, Inc. All rights reserved.

#include "device.h"
#include "memory.h"

namespace xrt::core::hip {

// Implementation
xrt_core::handle_map<mem_handle, std::shared_ptr<memory>> mem_cache;

} //namespace xrt::core::hip
