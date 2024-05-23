// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2024 Advanced Micro Device, Inc. All rights reserved.
#ifndef xrthip_module_h
#define xrthip_module_h

#include "common.h"
#include "context.h"
#include "experimental/xrt_ext.h"
#include "xrt/xrt_hw_context.h"
#include "xrt/xrt_kernel.h"

namespace xrt::core::hip {

// module_handle - opaque module handle
using module_handle = void*;

// function_handle - opaque function handle
using function_handle = void*;

// forward declaration
class function;

class module
{
  std::shared_ptr<context> m_ctx;
  bool m_xclbin_type;

public:
  module(std::shared_ptr<context> ctx, bool xclbin_type)
    : m_ctx{std::move(ctx)}
    , m_xclbin_type{xclbin_type}
  {}

  bool
  is_xclbin_module() const
  {
    return m_xclbin_type;
  }
};

class module_xclbin : public module
{
  xrt::xclbin m_xclbin;
  xrt::hw_context m_hw_ctx;
  xrt_core::handle_map<function_handle, std::shared_ptr<function>> function_cache;

public:
  module_xclbin(std::shared_ptr<context> ctx, const std::string& file_name);
  module_xclbin(std::shared_ptr<context> ctx, const void* image);

  void
  create_hw_context();

  xrt::kernel
  create_kernel(std::string& name);

  function_handle
  add_function(std::shared_ptr<function>&& f)
  {
    return insert_in_map(function_cache, f);
  }

  std::shared_ptr<function>
  get_function(function_handle handle)
  {
    return function_cache.get(handle);
  }

  xrt::hw_context&
  get_hw_context()
  {
    return m_hw_ctx;
  }
};

class module_xclbin : public module
{}

class function
{
  module* m_module;
  std::string m_func_name;
  //xrt::ext::kernel m_kernel;

public:
  function() = default;
  function(module_handle mod_hdl, std::string&& name);

  module*
  get_module() const
  {
    return m_module;
  }

  //xrt::ext::kernel&
  //get_kernel()
  //{
   // return m_kernel;
  //}

  std::string
  get_function_name() const
  {
    return m_func_name;
  }
};

extern xrt_core::handle_map<module_handle, std::shared_ptr<module>> module_cache;
} // xrt::core::hip

#endif

