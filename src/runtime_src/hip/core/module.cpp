// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Device, Inc. All rights reserved.

#include "hip/config.h"
#include "hip/hip_runtime_api.h"

#include "module.h"

namespace xrt::core::hip {
void
module_xclbin::
create_hw_context()
{
  auto xrt_dev = get_context()->get_xrt_device();
  auto uuid = xrt_dev.register_xclbin(m_xclbin);
  m_hw_ctx = xrt::hw_context{xrt_dev, uuid};
}

module_xclbin::
module_xclbin(std::shared_ptr<context> ctx, const std::string& file_name)
  : module(ctx, true)
{
  m_xclbin = xrt::xclbin{file_name};
  create_hw_context();
}

module_xclbin::
module_xclbin(std::shared_ptr<context> ctx, const void* image)
  : module(ctx, true)
{
  // we trust pointer sent by application and treat
  // it as xclbin data. Application can crash/seg fault
  // when improper data is passed
  m_xclbin = xrt::xclbin{static_cast<const axlf*>(image)};
  create_hw_context();
}

xrt::kernel
module_xclbin::
create_kernel(std::string& name)
{
  return xrt::kernel{m_hw_ctx, name};
}

module_elf::
module_elf(std::shared_ptr<context> ctx, const std::string& file_name)
  : module(ctx, false)
{
  m_elf = xrt::elf(file_name);
  m_module = xrt::module(m_elf);
}

module_elf::
module_elf(std::shared_ptr<context> ctx, const void* image)
  : module(ctx, false)
{
  throw std::runtime_error("module construction not supported");
}

function::
function(module_xclbin* mod_hdl, std::string&& name)
  : m_module{mod_hdl}
  , m_func_name{std::move(name)}
{
  if (!module_cache.count(mod_hdl))
    throw xrt_core::system_error(hipErrorInvalidResourceHandle, "module not available");

  m_kernel = m_module->create_kernel(m_func_name);
}

// Global map of modules
xrt_core::handle_map<module_handle, std::shared_ptr<module>> module_cache;
}

