/**
 * Copyright (C) 2022 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
#ifndef _XCL_AIE_PARSER_H_
#define _XCL_AIE_PARSER_H_

#include "xrt.h"
#include <boost/property_tree/ptree.hpp>
#include <stdint.h>
#include <vector>
#include <string>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int32_t i32;

// TODO: Add data driven mechanism to get these values
static const u32 SHIM_TILE_START = 0;
static const u32 NUM_SHIM_TILES = 1;
static const u32 MEM_TILE_START = 1;
static const u32 NUM_MEM_TILES = 1;
static const u32 CORE_TILE_START = 2;
static const u32 NUM_CORE_TILES = 4;
static const u32 NUM_CORE_DMA = 2;
static const u32 NUM_MEM_DMA = 6;
static const u32 NUM_SHIM_DMA = 2;
static const u32 NUM_MEM_LOCKS = 64;

namespace aie_parser {
/* Data structure to capture the dma status */
typedef struct {
  u32 s2mm_status;
  u32 mm2s_status;
} XAie_Dma_Status;

/* Data structure to capture the core tile status */
typedef struct {
  XAie_Dma_Status dma[NUM_CORE_DMA];
  u32 core_status;
  u32 program_counter;
  u32 stack_ptr;
  u32 link_reg;
} XAie_Core_Tile_Status;

/* Data structure to capture the mem tile status */
typedef struct {
  XAie_Dma_Status dma[NUM_MEM_DMA];
  u8 lock_value[NUM_MEM_LOCKS];
} XAie_Mem_Tile_Status;

/* Data structure to capture the shim tile status */
typedef struct {
  XAie_Dma_Status dma[NUM_SHIM_DMA];
} XAie_Shim_Tile_Status;

/* Data structure to capture column status */
typedef struct {
  XAie_Core_Tile_Status core_tile[NUM_CORE_TILES];
  XAie_Mem_Tile_Status mem_tile[NUM_MEM_TILES];
  XAie_Shim_Tile_Status shim_tile[NUM_SHIM_TILES];
} XAie_Col_Status;

enum class core_status : uint32_t
{
  XAIE_CORE_STATUS_ENABLE = 0U,
  XAIE_CORE_STATUS_RESET,
  XAIE_CORE_STATUS_MEMORY_STALL_S,
  XAIE_CORE_STATUS_MEMORY_STALL_W,
  XAIE_CORE_STATUS_MEMORY_STALL_N,
  XAIE_CORE_STATUS_MEMORY_STALL_E,
  XAIE_CORE_STATUS_LOCK_STALL_S,
  XAIE_CORE_STATUS_LOCK_STALL_W,
  XAIE_CORE_STATUS_LOCK_STALL_N,
  XAIE_CORE_STATUS_LOCK_STALL_E,
  XAIE_CORE_STATUS_STREAM_STALL_SSO,
  XAIE_CORE_STATUS_STREAM_STALL_MSO = 12U,
  XAIE_CORE_STATUS_CASCADE_STALL_SCD = 14U,
  XAIE_CORE_STATUS_CASCADE_STALL_MCD,
  XAIE_CORE_STATUS_DEBUG_HALT,
  XAIE_CORE_STATUS_ECC_ERROR_STALL,
  XAIE_CORE_STATUS_ECC_SCRUBBING_STALL,
  XAIE_CORE_STATUS_ERROR_HALT,
  XAIE_CORE_STATUS_CORE_DONE,
  XAIE_CORE_STATUS_CORE_PROC_BUS_STALL,
  XAIE_CORE_STATUS_MAX
};  
//std::vector<std::string> status_map(u32(core_status::XAIE_CORE_STATUS_MAX));

enum class dma_s2mm_status : uint32_t
{
  XAIE_DMA_STATUS_S2MM_STATUS = 0U,
  XAIE_DMA_STATUS_S2MM_STALLED_LOCK_ACK = 2U,
  XAIE_DMA_STATUS_S2MM_STALLED_LOCK_REL,
  XAIE_DMA_STATUS_S2MM_STALLED_STREAM_STARVATION,
  XAIE_DMA_STATUS_S2MM_STALLED_TCT_OR_COUNT_FIFO_FULL,
  XAIE_DMA_STATUS_S2MM_ERROR_BD_UNAVAIL = 10U,
  XAIE_DMA_STATUS_S2MM_ERROR_BD_INVALID,
  XAIE_DMA_STATUS_S2MM_ERROR_FOT_LENGTH,
  XAIE_DMA_STATUS_S2MM_ERROR_FOT_BDS_PER_TASK,
  XAIE_DMA_STATUS_S2MM_TASK_QUEUE_OVERFLOW = 18U,
  XAIE_DMA_STATUS_S2MM_CHANNEL_RUNNING,
  XAIE_DMA_STATUS_S2MM_TASK_QUEUE_SIZE,
  XAIE_DMA_STATUS_S2MM_CURRENT_BD = 24U,
  XAIE_DMA_STATUS_S2MM_MAX
};
//std::vector<std::string> dma_s2mm_map(u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_MAX));

enum class dma_mm2s_status : uint32_t
{
  XAIE_DMA_STATUS_MM2S_STATUS = 0U,
  XAIE_DMA_STATUS_MM2S_STALLED_LOCK_ACK = 2U,
  XAIE_DMA_STATUS_MM2S_STALLED_LOCK_REL,
  XAIE_DMA_STATUS_MM2S_STALLED_STREAM_BACKPRESSURE,
  XAIE_DMA_STATUS_MM2S_STALLED_TCT,
  XAIE_DMA_STATUS_MM2S_ERROR_BD_INVALID = 11U,
  XAIE_DMA_STATUS_MM2S_TASK_QUEUE_OVERFLOW = 18U,
  XAIE_DMA_STATUS_MM2S_CHANNEL_RUNNING,
  XAIE_DMA_STATUS_MM2S_TASK_QUEUE_SIZE,
  XAIE_DMA_STATUS_MM2S_CURRENT_BD = 24U,
  XAIE_DMA_STATUS_MM2S_MAX
};
//std::vector<std::string> dma_mm2s_map(u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_MAX));

boost::property_tree::ptree format_aie_info(aie_parser::XAie_Col_Status *col_status, uint32_t cols);

void
get_aie_rows_cols(xclDeviceHandle hdl, uint32_t &max_row, uint32_t &max_col);

void
get_aie_core_info(xclDeviceHandle hdl, aie_parser::XAie_Col_Status *col_status, uint32_t size, uint32_t &cols);

} // aie_parser
#endif