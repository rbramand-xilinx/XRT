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
  auto hip_ctx = get_current_context();
  throw_context_destroyed_if(!hip_ctx, "context is destroyed, no active context");

  std::lock_guard<std::mutex> lock(streams.mutex);
  return insert_in_map(streams.stream_cache, std::make_shared<stream>(hip_ctx, flags));
}

static void
hip_stream_destroy(hipStream_t stream)
{
  throw_invalid_handle_if(!stream, "stream is nullptr");
  throw_invalid_resource_if(stream == hipStreamPerThread, "Stream per thread can't be destroyed");

  std::lock_guard<std::mutex> lock(streams.mutex);
  streams.stream_cache.remove(stream);
}

static std::shared_ptr<stream>
get_stream(hipStream_t stream)
{
  // lock before getting any stream
  std::lock_guard<std::mutex> lock(streams.mutex);
  // app dint pass stream, use legacy default stream (null stream)
  if (!stream) {
    auto ctx = get_current_context();
    throw_context_destroyed_if(!ctx, "context is destroyed, no active context");
    return ctx->get_null_stream();
  }
  // TODO: Add support for per thread streams
  // if (stream == hipStreamPerThread)
  //   return get_per_thread_stream();

  return streams.stream_cache.get(stream);
}

static void
hip_stream_synchronize(hipStream_t stream)
{
  auto hip_stream = get_stream(stream);
  throw_invalid_handle_if(!hip_stream, "stream is invalid");
  hip_stream->synchronize();
  hip_stream->await_completion();
}

static void
hip_stream_wait_event(hipStream_t stream, hipEvent_t event, unsigned int flags)
{
  throw_invalid_handle_if(flags != 0, "flags should be 0");

  auto hip_wait_stream = get_stream(stream);
  throw_invalid_resource_if(!hip_stream, "stream is invalid");

  throw_invalid_handle_if(!event, "event is nullptr");
  auto hip_event = event_cache.get(event);
  throw_invalid_resource_if(!hip_event, "event is invalid");
  throw_if(!hip_event->is_recorded(), hipErrorStreamCaptureIsolation, "Event passed is not recorded");
  auto hip_event_stream = hip_event->get_stream();

  if (hip_wait_stream == hip_event_stream) {
    hip_wait_stream->record_top_event(hip_event.get());
  }
  else {
    // create dumm_event with wait stream and parent as current event
    auto dummy_event = std::make_shared<dummy_event>(hip_wait_stream, hip_event);
    hip_wait_stream->record_top_event(dummy_event.get());
  }
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

