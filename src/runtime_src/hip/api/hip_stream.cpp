// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2023-2024 Advanced Micro Device, Inc. All rights reserved.

#include "hip/core/common.h"
#include "hip/core/stream.h"

namespace xrt::core::hip {
// In Hip, based on flags we can create default or non blocking streams.
// If application doesn't explicitly specify stream we use default stream
// for such operations. The default stream also has two modes legacy or per thread.
// Legacy default stream is also called as NULL stream. Null stream waits on all explictly
// created default streams in the same context when an operation is enqueued and explicitly
// created default streams wait on null stream in that context.
// Per thread stream is also default stream but is created per thread per context and waits
// on null stream of that context.

static stream_handle
hip_stream_create_with_flags(unsigned int flags)
{
  throw_invalid_value_if(flags != hipStreamDefault && flags != hipStreamNonBlocking,
                         "Invalid flags passed for stream creation");

  auto hip_stream = std::make_shared<stream>(get_current_context(), flags);
  auto handle = hip_stream.get();
  stream_cache.add(handle, std::move(hip_stream));
  return handle;
}

static void
hip_stream_destroy(hipStream_t stream)
{
  throw_invalid_handle_if(!stream, "stream is nullptr");
  throw_invalid_resource_if(stream == hipStreamPerThread, "Stream per thread can't be destroyed");

  stream_cache.remove(stream);
}

static void
hip_stream_synchronize(hipStream_t stream)
{
  throw std::runtime_error("Not implemented");
}

static void
hip_stream_wait_event(hipStream_t stream, hipEvent_t event, unsigned int flags)
{
  throw_invalid_handle_if(!event, "event is nullptr");

  throw std::runtime_error("Not implemented");
}
} // // xrt::core::hip

// =========================================================================
// Stream related apis implementation
hipError_t
hipStreamCreateWithFlags(hipStream_t* stream, unsigned int flags)
{
  try {
    throw_invalid_value_if(!stream, "stream passed is nullptr");

    auto handle = xrt::core::hip::hip_stream_create_with_flags(flags);
    *stream = reinterpret_cast<hipStream_t>(handle);
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
hipStreamDestroy(hipStream_t stream)
{
  try {
    xrt::core::hip::hip_stream_destroy(stream);
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
hipStreamSynchronize(hipStream_t stream)
{
  try {
    xrt::core::hip::hip_stream_synchronize(stream);
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
hipStreamWaitEvent(hipStream_t stream, hipEvent_t event, unsigned int flags)
{
  try {
    xrt::core::hip::hip_stream_wait_event(stream, event, flags);
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

