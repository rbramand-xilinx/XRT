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

#include "aie_parser.h"

namespace aie_parser {
  namespace bpt = boost::property_tree;

  std::vector<std::string> status_map;
  std::vector<std::string> dma_s2mm_map;
  std::vector<std::string> dma_mm2s_map;
  static void
  initialize_mapping()
  {
    // core status
    status_map.resize(u32(core_status::XAIE_CORE_STATUS_MAX));
    status_map[u32(core_status::XAIE_CORE_STATUS_ENABLE)] = "Enable";
    status_map[u32(core_status::XAIE_CORE_STATUS_RESET)] = "Reset";
    status_map[u32(core_status::XAIE_CORE_STATUS_MEMORY_STALL_S)] = "Memory_Stall_S";
    status_map[u32(core_status::XAIE_CORE_STATUS_MEMORY_STALL_W)] = "Memory_Stall_W";
    status_map[u32(core_status::XAIE_CORE_STATUS_MEMORY_STALL_N)] = "Memory_Stall_N";
    status_map[u32(core_status::XAIE_CORE_STATUS_MEMORY_STALL_E)] = "Memory_Stall_E";
    status_map[u32(core_status::XAIE_CORE_STATUS_LOCK_STALL_S)] = "Lock_Stall_S";
    status_map[u32(core_status::XAIE_CORE_STATUS_LOCK_STALL_W)] = "Lock_Stall_W";
    status_map[u32(core_status::XAIE_CORE_STATUS_LOCK_STALL_N)] = "Lock_Stall_N";
    status_map[u32(core_status::XAIE_CORE_STATUS_LOCK_STALL_E)] = "Lock_Stall_E";
    status_map[u32(core_status::XAIE_CORE_STATUS_STREAM_STALL_SSO)] = "Stream_Stall_SSO";
    status_map[u32(core_status::XAIE_CORE_STATUS_STREAM_STALL_MSO)] = "Stream_Stall_MSO";
    status_map[u32(core_status::XAIE_CORE_STATUS_CASCADE_STALL_SCD)] = "Cascade_Stall_SCD";
    status_map[u32(core_status::XAIE_CORE_STATUS_CASCADE_STALL_MCD)] = "Cascade_Stall_MCD";
    status_map[u32(core_status::XAIE_CORE_STATUS_DEBUG_HALT)] = "Debug_Halt";
    status_map[u32(core_status::XAIE_CORE_STATUS_ECC_ERROR_STALL)] = "ECC_Error_Stall";
    status_map[u32(core_status::XAIE_CORE_STATUS_ECC_SCRUBBING_STALL)] = "ECC_Scrubbing_Stall";
    status_map[u32(core_status::XAIE_CORE_STATUS_ERROR_HALT)] = "Error_Halt";
    status_map[u32(core_status::XAIE_CORE_STATUS_CORE_DONE)] = "Core_Done";
    status_map[u32(core_status::XAIE_CORE_STATUS_CORE_PROC_BUS_STALL)] = "Core_Proc_Bus_Stall";

    // DMA s2mm status
    dma_s2mm_map.resize(u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_MAX));
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_STATUS)] = "Status";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_STALLED_LOCK_ACK)] = "Stalled_Lock_Acq";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_STALLED_LOCK_REL)] = "Stalled_Lock_Rel";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_STALLED_STREAM_STARVATION)] = "Stalled_Stream_Starvation";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_STALLED_TCT_OR_COUNT_FIFO_FULL)] = "Stalled_TCT_Or_Count_FIFO_Full";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_ERROR_BD_UNAVAIL)] = "Error_BD_Unavail";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_ERROR_BD_INVALID)] = "Error_BD_Invalid";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_ERROR_FOT_LENGTH)] = "Error_FoT_Length";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_ERROR_FOT_BDS_PER_TASK)] = "Error_Fot_BDs";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_TASK_QUEUE_OVERFLOW)] = "Task_Queue_Overflow";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_CHANNEL_RUNNING)] = "Channel_Running";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_TASK_QUEUE_SIZE)] = "Task_Queue_Size";
    dma_s2mm_map[u32(dma_s2mm_status::XAIE_DMA_STATUS_S2MM_CURRENT_BD)] = "Cur_BD";

    // DMA mm2s status
    dma_mm2s_map.resize(u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_MAX));
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_STATUS)] = "Status";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_STALLED_LOCK_ACK)] = "Stalled_Lock_Acq";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_STALLED_LOCK_REL)] = "Stalled_Lock_Rel";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_STALLED_STREAM_BACKPRESSURE)] = "Stalled_Stream_Back_Pressure";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_STALLED_TCT)] = "Stalled_TCT";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_ERROR_BD_INVALID)] = "Error_BD_Invalid";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_TASK_QUEUE_OVERFLOW)] = "Task_Queue_Overflow";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_CHANNEL_RUNNING)] = "Channel_Running";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_TASK_QUEUE_SIZE)] = "Task_Queue_Size";
    dma_mm2s_map[u32(dma_mm2s_status::XAIE_DMA_STATUS_MM2S_CURRENT_BD)] = "Cur_BD";
  }

  struct initialize { 
    initialize() { initialize_mapping(); }
  };
  static initialize obj;

  static std::string dma_channel_status_to_string(u32 status)
  {
    u32 count = 0;

    while (status) {
      if (status & 0x1)
        return dma_s2mm_map[count];
      
      status >>= 1;
      count++;
    }

    return "unknown";
  }

  static void
  populate_dma(aie_parser:: XAie_Core_Tile_Status &core, bpt::ptree &dma)
  {
    bpt::ptree channel_status_mm2s_array;
    bpt::ptree channel_status_s2mm_array;
    bpt::ptree queue_size_mm2s_array;
    bpt::ptree queue_size_s2mm_array;
    bpt::ptree queue_status_mm2s_array;
    bpt::ptree queue_status_s2mm_array;
    bpt::ptree current_bd_mm2s_array;
    bpt::ptree current_bd_s2mm_array;

    for(u32 i = 0; i < NUM_CORE_DMA; i++)
    {
      // channel status
      bpt::ptree channel_status_mm2s;
      bpt::ptree channel_status_s2mm;
      channel_status_mm2s.put("", dma_channel_status_to_string(core.dma[i].mm2s_status));
      channel_status_s2mm.put("", dma_channel_status_to_string(core.dma[i].s2mm_status));

      channel_status_mm2s_array.push_back(std::make_pair("", channel_status_mm2s));
      channel_status_s2mm_array.push_back(std::make_pair("", channel_status_s2mm)); 

      // fill dummy data for queue_size, queue_status, current bd
      // fill proper info once driver makes it available
      queue_size_mm2s_array.push_back(std::make_pair("", bpt::ptree(""))); 
      queue_size_s2mm_array.push_back(std::make_pair("", bpt::ptree("")));

      queue_status_mm2s_array.push_back(std::make_pair("", bpt::ptree(""))); 
      queue_status_s2mm_array.push_back(std::make_pair("", bpt::ptree("")));

      current_bd_mm2s_array.push_back(std::make_pair("", bpt::ptree(""))); 
      current_bd_s2mm_array.push_back(std::make_pair("", bpt::ptree("")));
    }

    dma.add_child("channel_status.mm2s", channel_status_mm2s_array);
    dma.add_child("channel_status.s2mm", channel_status_s2mm_array);

    dma.add_child("queue_size.mm2s", queue_size_mm2s_array);
    dma.add_child("queue_size.s2mm", queue_size_s2mm_array);

    dma.add_child("queue_status.mm2s", queue_status_mm2s_array);
    dma.add_child("queue_status.s2mm", queue_status_s2mm_array);

    dma.add_child("current_bd.mm2s", current_bd_mm2s_array);
    dma.add_child("current_bd.s2mm", current_bd_s2mm_array);
  }

  static void 
  core_status_to_string_array(u32 status, std::vector<std::string> &status_vec)
  {
    u32 count = 0;

    while (status) {
      if (status & 0x1)
        status_vec.push_back(status_map[count]);
    
      status >>= 1;
      count++;
    }
  }

  static void 
  get_core_tile_info(aie_parser:: XAie_Core_Tile_Status &core, bpt::ptree &pt)
  {
    bpt::ptree core_pt;
    bpt::ptree status_array;
    bpt::ptree tmp;
    bpt::ptree tmp_array;
    std::vector<std::string> status_vec;
    
    core_status_to_string_array(core.core_status, status_vec);
    for (auto &x : status_vec) {
      bpt::ptree status_pt;
      status_pt.put("",x);
      status_array.push_back(std::make_pair("",status_pt));
    }
    core_pt.add_child("status", status_array);

    // fill program counter as array
    tmp.put("", core.program_counter);
    tmp_array.push_back(std::make_pair("", tmp));
    core_pt.add_child("pc", tmp_array);

    // fill stack pointer as array
    tmp_array = {};
    tmp.put("", core.stack_ptr);
    tmp_array.push_back(std::make_pair("", tmp));
    core_pt.add_child("sp", tmp_array);

    // fill empty value for link register so that upper common layer will not throw
    tmp_array = {};
    tmp.put("", core.link_reg);
    tmp_array.push_back(std::make_pair("", tmp));
    core_pt.add_child("lr", tmp_array);

    pt.add_child("core", core_pt);

    // fill DMA status
    bpt::ptree dma_pt;
    populate_dma(core, dma_pt);
    pt.add_child("dma", dma_pt);
  }

  bpt::ptree format_aie_info(aie_parser::XAie_Col_Status *col_status, uint32_t cols)
  {
    bpt::ptree pt_array;
    aie_parser::XAie_Col_Status *col_itr = col_status;

    for (uint32_t col = 0; col < cols; col++) {
        // row 0 is shim tile
        // row 1 is mem tile
        // core tiles are from row 2
        for (uint32_t row = 0; row < NUM_CORE_TILES; row++) {
          bpt::ptree pt;
          pt.put("col", col);
          pt.put("row", row + CORE_TILE_START);

          get_core_tile_info(col_itr[col].core_tile[row], pt);

          pt_array.push_back(std::make_pair(std::to_string(col)+"_"+std::to_string(row + CORE_TILE_START), pt));
        }
    }

    bpt::ptree pt_aie_core;
    pt_aie_core.add_child("aie_core",pt_array);  
    return pt_aie_core;
  }
}