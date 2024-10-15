// Copyright (C) 2023 Advanced Micro Devices, Inc. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#define XCL_DRIVER_DLL_EXPORT  // exporting xrt_module.h
#define XRT_API_SOURCE         // exporting xrt_module.h
#define XRT_CORE_COMMON_SOURCE // in same dll as core_common
#include "core/common/config_reader.h"
#include "core/common/message.h"
#include "experimental/xrt_module.h"
#include "experimental/xrt_elf.h"
#include "experimental/xrt_ext.h"

#include "xrt/xrt_bo.h"
#include "xrt/xrt_hw_context.h"
#include "xrt/xrt_uuid.h"

#include "elf_int.h"
#include "ert.h"
#include "module_int.h"
#include "core/common/debug.h"
#include "core/common/error.h"

#include <boost/format.hpp>
#include <elfio/elfio.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <cxxabi.h>
#include <numeric>
#include <map>
#include <set>
#include <string>
#include <sstream>

#ifndef AIE_COLUMN_PAGE_SIZE
# define AIE_COLUMN_PAGE_SIZE 8192  // NOLINT
#endif

namespace
{

// Control code is padded to page size, where page size is
// 0 if no padding is required.   The page size should be
// embedded as ELF metadata in the future.
static constexpr size_t column_page_size = AIE_COLUMN_PAGE_SIZE;
static constexpr uint8_t Elf_Amd_Aie2p        = 69;
static constexpr uint8_t Elf_Amd_Aie2ps       = 64;
static constexpr uint8_t Elf_Amd_Aie2p_config = 70;

static const char* Scratch_Pad_Mem_Symbol = "scratch-pad-mem";
static const char* Control_Packet_Symbol = "control-packet";
//static const char* pdi_symbol = "pdi0";
static const char* pdi_symbol = ".pdi.0";

struct buf
{
  std::vector<uint8_t> m_data;

  void
  append_section_data(ELFIO::section* sec)
  {
    auto sz = sec->get_size();
    auto sdata = sec->get_data();
    m_data.insert(m_data.end(), sdata, sdata + sz);
  }

  [[nodiscard]] size_t
  size() const
  {
    return m_data.size();
  }

  [[nodiscard]] const uint8_t*
  data() const
  {
    return m_data.data();
  }

  void
  append_section_data(const uint8_t* userptr, size_t sz)
  {
    m_data.insert(m_data.end(), userptr, userptr + sz);
  }

  void
  pad_to_page(uint32_t page)
  {
    if (!column_page_size)
      return;

    auto pad = (page + 1) * column_page_size;

    if (m_data.size() > pad)
      throw std::runtime_error("Invalid ELF section size");

    m_data.resize(pad);
  }
};

using instr_buf = buf;
using control_packet = buf;
using ctrlcode = buf; // represent control code for column or partition

// struct patcher - patcher for a symbol
//
// Manage patching of a symbol in the control code.  The symbol
// type is used to determine the patching method.
//
// The patcher is created with an offset into a buffer object
// representing the contigous control code for all columns
// and pages. The base address of the buffer object is passed
// in as a parameter to patch().
struct patcher
{
  enum class symbol_type {
    uc_dma_remote_ptr_symbol_kind = 1,
    shim_dma_base_addr_symbol_kind = 2,      // patching scheme needed by AIE2PS firmware
    scalar_32bit_kind = 3,
    control_packet_48 = 4,                   // patching scheme needed by firmware to patch control packet
    shim_dma_48 = 5,                         // patching scheme needed by firmware to patch instruction buffer
    shim_dma_aie4_base_addr_symbol_kind = 6, // patching scheme needed by AIE4 firmware
    address_64 = 7,                          // patch pdi address
    unknown_symbol_kind = 8
  };

  enum class buf_type {
    ctrltext = 0,   // control code
    ctrldata = 1,       // control packet
    preempt_save = 2,   // preempt_save
    preempt_restore = 3, // preempt_restore
    pdi = 4, // pdi
    buf_type_count = 5   // total number of buf types
  };

  inline static const char*
  section_name_to_string(buf_type bt)
  {
    static const char* Section_Name_Array[static_cast<int>(buf_type::buf_type_count)] = { ".ctrltext",
                                                                                          ".ctrldata",
                                                                                          ".preempt_save",
                                                                                          ".preempt_restore",
                                                                                          ".pdi"};

    return Section_Name_Array[static_cast<int>(bt)];
  }

  buf_type m_buf_type = buf_type::ctrltext;
  symbol_type m_symbol_type = symbol_type::shim_dma_48;

  struct patch_info {
    uint64_t offset_to_patch_buffer;
    uint32_t offset_to_base_bo_addr;
    uint32_t mask; // This field is valid only when patching scheme is scalar_32bit_kind
  };

  std::vector<patch_info> m_ctrlcode_patchinfo;

  patcher(symbol_type type, std::vector<patch_info> ctrlcode_offset, buf_type t)
    : m_buf_type(t)
    , m_symbol_type(type)
    , m_ctrlcode_patchinfo(std::move(ctrlcode_offset))
  {}

  void
  patch_64bitaddr(uint32_t* data_to_patch, uint64_t addr)
  {
    *data_to_patch = static_cast<uint32_t>(addr & 0xffffffff);
    *(data_to_patch + 1) = static_cast<uint32_t>((addr >> 32) & 0xffffffff);
  }
  
// Replace certain bits of *data_to_patch with register_value. Which bits to be replaced is specified by mask
  // For     *data_to_patch be 0xbb11aaaa and mask be 0x00ff0000
  // To make *data_to_patch be 0xbb55aaaa, register_value must be 0x00550000
  void
  patch32(uint32_t* data_to_patch, uint64_t register_value, uint32_t mask)
  {
    if ((reinterpret_cast<uintptr_t>(data_to_patch) & 0x3) != 0)
      throw std::runtime_error("address is not 4 byte aligned for patch32");

    auto new_value = *data_to_patch;
    new_value = (new_value & ~mask) | (register_value & mask);
    *data_to_patch = new_value;
  }

  void
  patch57(uint32_t* bd_data_ptr, uint64_t patch)
  {
    uint64_t base_address =
      ((static_cast<uint64_t>(bd_data_ptr[8]) & 0x1FF) << 48) |                       // NOLINT
      ((static_cast<uint64_t>(bd_data_ptr[2]) & 0xFFFF) << 32) |                      // NOLINT
      bd_data_ptr[1];

    base_address += patch;
    bd_data_ptr[1] = (uint32_t)(base_address & 0xFFFFFFFF);                           // NOLINT
    bd_data_ptr[2] = (bd_data_ptr[2] & 0xFFFF0000) | ((base_address >> 32) & 0xFFFF); // NOLINT
    bd_data_ptr[8] = (bd_data_ptr[8] & 0xFFFFFE00) | ((base_address >> 48) & 0x1FF);  // NOLINT
  }

  void
  patch57_aie4(uint32_t* bd_data_ptr, uint64_t patch)
  {
    constexpr uint64_t ddr_aie_addr_offset = 0x80000000;

    uint64_t base_address =
      ((static_cast<uint64_t>(bd_data_ptr[0]) & 0x1FFFFFF) << 32) |                   // NOLINT
      bd_data_ptr[1];

    base_address += patch + ddr_aie_addr_offset;  //2G offset
    bd_data_ptr[1] = (uint32_t)(base_address & 0xFFFFFFFF);                           // NOLINT
    bd_data_ptr[0] = (bd_data_ptr[0] & 0xFE000000) | ((base_address >> 32) & 0x1FFFFFF);// NOLINT
  }

  void
  patch_ctrl48(uint32_t* bd_data_ptr, uint64_t patch)
  {
    // This patching scheme is originated from NPU firmware
    constexpr uint64_t ddr_aie_addr_offset = 0x80000000;

    uint64_t base_address =
      ((static_cast<uint64_t>(bd_data_ptr[3]) & 0xFFF) << 32) |                       // NOLINT
      ((static_cast<uint64_t>(bd_data_ptr[2])));

    base_address = base_address + patch + ddr_aie_addr_offset;
    bd_data_ptr[2] = (uint32_t)(base_address & 0xFFFFFFFC);                           // NOLINT
    bd_data_ptr[3] = (bd_data_ptr[3] & 0xFFFF0000) | (base_address >> 32);            // NOLINT
  }

  void patch_shim48(uint32_t* bd_data_ptr, uint64_t patch)
  {
    std::cout << "[rahul] : inside " << __func__ << " patching addr 0x" << std::hex << bd_data_ptr << " and value " << std::hex << patch << std::endl;
    // This patching scheme is originated from NPU firmware
    constexpr uint64_t ddr_aie_addr_offset = 0x80000000;

    uint64_t base_address =
      ((static_cast<uint64_t>(bd_data_ptr[2]) & 0xFFF) << 32) |                       // NOLINT
      ((static_cast<uint64_t>(bd_data_ptr[1])));

    std::cout << "--- base addr before adding patch - " << std::hex << base_address << std::endl;

    base_address = base_address + patch + ddr_aie_addr_offset;
    std::cout << "--- base addr after adding patch - " << std::hex << base_address << std::endl;
    std::cout << "[rahul] : before bd_data_ptr[1] - " << std::hex << bd_data_ptr[1] << std::endl;
    std::cout << "[rahul] : before bd_data_ptr[2] - " << std::hex << bd_data_ptr[2] << std::endl;
    bd_data_ptr[1] = (uint32_t)(base_address & 0xFFFFFFFC);                           // NOLINT
    bd_data_ptr[2] = (bd_data_ptr[2] & 0xFFFF0000) | (base_address >> 32);            // NOLINT
    std::cout << "[rahul] : after bd_data_ptr[1] - " << std::hex << bd_data_ptr[1] << std::endl;
    std::cout << "[rahul] : after bd_data_ptr[2] - " << std::hex << bd_data_ptr[2] << std::endl;
  }

  void
  patch(uint8_t* base, uint64_t new_value)
  {
    for (auto item : m_ctrlcode_patchinfo) {
      auto bd_data_ptr = reinterpret_cast<uint32_t*>(base + item.offset_to_patch_buffer);
      std::cout << "[rahul] : inside - " << __func__ << "base - " << std::hex << " bd_data_ptr - " << std::hex << bd_data_ptr << std::endl;
      std::cout << "--- inside patch of patcher bd_data_ptr : " << std::hex << bd_data_ptr << std::endl;
      switch (m_symbol_type) {
      case symbol_type::address_64:
          // new_value is a 64bit address
          patch_64bitaddr(bd_data_ptr, new_value);
        break;
      case symbol_type::scalar_32bit_kind:
        // new_value is a register value
        if (item.mask)
          patch32(bd_data_ptr, new_value, item.mask);
        break;
      case symbol_type::shim_dma_base_addr_symbol_kind:
        // new_value is a bo address
        patch57(bd_data_ptr, new_value + item.offset_to_base_bo_addr);
        break;
      case symbol_type::shim_dma_aie4_base_addr_symbol_kind:
        // new_value is a bo address
        patch57_aie4(bd_data_ptr, new_value + item.offset_to_base_bo_addr);
        break;
      case symbol_type::control_packet_48:
        // new_value is a bo address
        patch_ctrl48(bd_data_ptr, new_value + item.offset_to_base_bo_addr);
        break;
      case symbol_type::shim_dma_48:
        // new_value is a bo address
	std::cout << "[rahul] : new_value - " << new_value << " offset - " << std::hex << item.offset_to_base_bo_addr << std::endl;
        patch_shim48(bd_data_ptr, new_value + item.offset_to_base_bo_addr);
        break;
      default:
        throw std::runtime_error("Unsupported symbol type");
      }
    }
  }
};

XRT_CORE_UNUSED void
dump_bo(xrt::bo& bo, const std::string& filename)
{
  std::ofstream ofs(filename, std::ios::out | std::ios::binary);
  if (!ofs.is_open())
    throw std::runtime_error("Failure opening file " + filename + " for writing!");

  auto buf = bo.map<char*>();
  ofs.write(buf, bo.size());
}

XRT_CORE_UNUSED std::string
generate_key_string(const std::string& argument_name, patcher::buf_type type, uint32_t index)
{
  std::string buf_string = std::to_string(static_cast<int>(type));
  return argument_name + buf_string + std::to_string(index);
}

static std::string
demangle(const std::string& mangled_name)
{
  int status = 0;
  char* demangled_name = abi::__cxa_demangle(mangled_name.c_str(), nullptr, nullptr,  &status);

  if (status)
    throw std::runtime_error("Error demangling kernel signature");

  std::string result {demangled_name};
  std::free(demangled_name); // Free the allocated memory
  return result;
}

} // namespace

namespace xrt
{
// class module_impl - Base class for different implementations
class module_impl
{
  xrt::uuid m_cfg_uuid;   // matching hw configuration id

public:
  explicit module_impl(xrt::uuid cfg_uuid)
    : m_cfg_uuid(std::move(cfg_uuid))
  {}

  explicit module_impl(const module_impl* parent)
    : m_cfg_uuid(parent->m_cfg_uuid)
  {}

  virtual ~module_impl() = default;

  module_impl() = delete;
  module_impl(const module_impl&) = delete;
  module_impl(module_impl&&) = delete;
  module_impl& operator=(const module_impl&) = delete;
  module_impl& operator=(module_impl&&) = delete;

  [[nodiscard]] xrt::uuid
  get_cfg_uuid() const
  {
    return m_cfg_uuid;
  }

  // Get raw instruction buffer data for all columns or for
  // single partition.  The returned vector has the control
  // code as extracted from ELF or userptr.
  [[nodiscard]] virtual const std::vector<ctrlcode>&
  get_data() const
  {
    throw std::runtime_error("Not supported");
  }

  [[nodiscard]] virtual std::pair<uint32_t, instr_buf>
  get_instr(uint32_t index = 0) const
  {
    throw std::runtime_error("Not supported");
  }

  [[nodiscard]] virtual std::pair<uint32_t, buf>
  get_preempt_save() const
  {
    throw std::runtime_error("Not supported");
  }

  [[nodiscard]] virtual std::pair<uint32_t, buf>
  get_preempt_restore() const
  {
    throw std::runtime_error("Not supported");
  }

  [[nodiscard]] virtual std::pair<uint32_t, buf>
  get_pdi(uint32_t index = 0) const
  {
    throw std::runtime_error("Not supported");
  }

  [[nodiscard]] virtual size_t
  get_scratch_pad_mem_size() const
  {
    throw std::runtime_error("Not supported");
  }

  [[nodiscard]] virtual std::pair<uint32_t, control_packet>
  get_ctrlpkt(uint32_t index = 0) const
  {
    throw std::runtime_error("Not supported");
  }

  [[nodiscard]] virtual xrt::bo&
  get_scratch_pad_mem()
  {
    throw std::runtime_error("Not supported");
  }

  [[nodiscard]] virtual xrt::hw_context
  get_hw_context() const
  {
    return {};
  }

  // Fill in ERT command payload in ELF flow. The payload is after
  // extra_cu_mask and before CU arguments. Return the current point of
  // the ERT command payload
  virtual uint32_t*
  fill_ert_dpu_data(uint32_t *) const
  {
    throw std::runtime_error("Not supported");
  }

  //virtual uint32_t*
  //fill_ert_dpu_data_elf_flow(uint32_t *) const
  //{
  //  throw std::runtime_error("Not supported");
  //}

  [[nodiscard]] virtual uint8_t
  get_os_abi() const
  {
    throw std::runtime_error("Not supported");
  }

  // Patch ctrlcode buffer object for global argument
  //
  // @param bo_ctrlcode - bo containing ctrlcode
  // @param symbol - symbol name
  // @param index - argument index
  // @param bo - global argument to patch into ctrlcode
  // @param buf_type - whether it is control-code, control-packet, preempt-save or preempt-restore
  // @param sec_index - index of section to be patched
  virtual void
  patch_instr(xrt::bo&, const std::string&, size_t, const xrt::bo&, patcher::buf_type, uint32_t)
  {
    throw std::runtime_error("Not supported ");
  }

  // Patch ctrlcode buffer object for global argument
  //
  // @param argname - argument name
  // @param index - argument index
  // @param bo - global argument to patch into ctrlcode
  virtual void
  patch(const std::string&, size_t, const xrt::bo&)
  {
    throw std::runtime_error("Not supported");
  }

  // Patch ctrlcode buffer object for scalar argument
  //
  // @param symbol - symbol name
  // @param value - patch value
  // @param size - size of patch value
  virtual void
  patch(const std::string&, size_t, const void*, size_t)
  {
    throw std::runtime_error("Not supported");
  }

  // Patch symbol in control code with patch value
  //
  // @param base - base address of control code buffer object
  // @param symbol - symbol name
  // @param index - argument index
  // @param patch - patch value
  // @param buf_type - whether it is control-code, control-packet, preempt-save or preempt-restore
  // @param sec_index - index of section to be patched
  // @Return true if symbol was patched, false otherwise
  virtual bool
  patch(uint8_t*, const std::string&, size_t, uint64_t, patcher::buf_type, uint32_t)
  {
    throw std::runtime_error("Not supported");
  }

  // Get the number of patchers for arguments.  The returned
  // value is the number of arguments that must be patched before
  // the control code can be executed.
  [[nodiscard]] virtual size_t
  number_of_arg_patchers() const
  {
    return 0;
  }

  // Check that all arguments have been patched and sync control code
  // buffer if necessary.  Throw if not all arguments have been patched.
  virtual void
  sync_if_dirty()
  {
    throw std::runtime_error("Not supported");
  }

  // Get the ERT command opcode in ELF flow.
  virtual ert_cmd_opcode
  get_ert_opcode() const
  {
    throw std::runtime_error("Not supported");
  }

  // get partition size if elf has the info
  [[nodiscard]] virtual uint32_t
  get_partition_size() const
  {
    throw std::runtime_error("Not supported");
  }

  // get kernel signature in mangled format
  [[nodiscard]] virtual std::string
  get_kernel_signature() const
  {
    throw std::runtime_error("Not supported");
  }

  std::string
  get_demangled_kernel_signature() const
  {
    // Gets kernel signature or throws error if not applicable
  
    // TODO : add code to get kernel signature
    // Use default signature for now
    std::string kernel_sig {"_Z3DPUPvS_S_S_S_"};

    // demangle and return
    return demangle(kernel_sig);
  }

  std::string
  get_kernel_name() const
  {
    std::string demangled_name = get_demangled_kernel_signature();

    // extract kernel name
    size_t pos = demangled_name.find('(');
    if (pos == std::string::npos)
      throw std::runtime_error("Failed to get kernel name");

    return demangled_name.substr(0, pos);
  }
};

// class module_elf - Elf provided by application
//
// Construct a module from a ELF file provided by the application.
//
// The ELF is mined for control code sections which are extracted
// into a vector of ctrlcode objects.  The ctrlcode objects are
// binary blobs that are used to populate the ert_dpu_data elements
// of an ert_packet.
//
// The ELF is also mined for relocation sections which are used to to
// patch the control code with the address of a buffer object or value
// of a scalar object used as an argument. The relocations are used to
// construct patcher objects for each argument.
class module_elf : public module_impl
{
  // rela->addend have offset to base-bo-addr info along with schema
  // [0:3] bit are used for patching schema, [4:31] used for base-bo-addr
  constexpr static uint32_t addend_shift = 4;
  constexpr static uint32_t addend_mask = ~((uint32_t)0) << addend_shift;
  constexpr static uint32_t schema_mask = ~addend_mask;

  const ELFIO::elfio& m_elfio; // we should not modify underlying elf
  uint8_t m_os_abi = Elf_Amd_Aie2p;
  std::map<std::string, patcher> m_arg2patcher;

  std::vector<ctrlcode> m_ctrlcodes;

  // New ELFs of Aie2p contain multiple ctrltext, ctrldata  and PDI sections
  // sections will be of format .ctrltext.* where .* has index of that section type
  // Below maps has this index as key and value is pair of <section index, data buffer>
  std::map<uint32_t, std::pair<uint32_t, instr_buf>> m_instr_buf_map;
  std::map<uint32_t, std::pair<uint32_t, control_packet>> m_ctrl_packet_map;
  std::map<uint32_t, std::pair<uint32_t, buf>> m_pdi_buf_map;

  buf m_save_buf;
  uint32_t m_save_buf_sec_idx = UINT32_MAX;
  bool m_save_buf_exist = false;

  buf m_restore_buf;
  uint32_t m_restore_buf_sec_idx = UINT32_MAX;
  bool m_restore_buf_exist = false;
  
  size_t m_scratch_pad_mem_size = 0;
  uint32_t m_partition_size = UINT32_MAX;
  std::string m_kernel_signature;

  // The ELF sections embed column and page information in their
  // names.  Extract the column and page information from the
  // section name, default to single column and page when nothing
  // is specified.
  static std::pair<uint32_t, uint32_t>
  get_column_and_page(const std::string& name)
  {
    constexpr size_t first_dot = 9;  // .ctrltext.<col>.<page>
    auto dot1 = name.find_first_of(".", first_dot);
    auto dot2 = name.find_first_of(".", first_dot + 1);
    auto col = dot1 != std::string::npos
      ? std::stoi(name.substr(dot1 + 1, dot2))
      : 0;
    auto page = dot2 != std::string::npos
      ? std::stoi(name.substr(dot2 + 1))
      : 0;
    return { col, page };
  }

  uint32_t
  get_section_name_index(const std::string& name)
  {
    // Elf_Amd_Aie2p has sections .sec_name
    // Elf_Amd_Aie2p_config has sections .sec_name.*
    auto pos = name.find_last_of(".");
    return (pos == 0) ? 0 : std::stoul(name.substr(pos + 1, 1));
  }

  void
  initialize_partition_size()
  {
    static constexpr const char* partition_section_name {".note.xrt.configuration"};
    // note 0 in .note.xrt.configuration section has partition size
    static constexpr ELFIO::Elf_Word partition_note_num = 0;

    auto partition_section = m_elfio.sections[partition_section_name];
    if (!partition_section)
      return; // elf doesn't have partition info section, partition size holds UINT32_MAX

    ELFIO::note_section_accessor accessor(m_elfio, partition_section);
    ELFIO::Elf_Word type;
    std::string name;
    char* desc;
    ELFIO::Elf_Word desc_size;
    if (!accessor.get_note(partition_note_num, type, name, desc, desc_size))
      throw std::runtime_error("Failed to get partition info, partition note not found\n");
    m_partition_size = std::stoul(std::string{static_cast<char*>(desc), desc_size});
  }

  void
  initialize_kernel_signature()
  {
    static constexpr const char* symtab_section_name {".symtab"};

    ELFIO::section* symtab = m_elfio.sections[symtab_section_name];
    if (!symtab)
      return; // elf doesn't have .symtab section, kernel_signature will be empty string

    // Get the symbol table
    const ELFIO::symbol_section_accessor symbols(m_elfio, symtab);
    // Iterate over all symbols
    for (ELFIO::Elf_Xword i = 0; i < symbols.get_symbols_num(); ++i) {
      std::string name;
      ELFIO::Elf64_Addr value;
      ELFIO::Elf_Xword size;
      unsigned char bind;
      unsigned char type;
      ELFIO::Elf_Half section_index;
      unsigned char other;

      // Read symbol data
      symbols.get_symbol(i, name, value, size, bind, type, section_index, other);

      // there will be only 1 kernel signature symbol entry in .symtab section whose
      // type is FUNC
      if (type == ELFIO::STT_FUNC) {
        m_kernel_signature = demangle(name);
        break;
      }
    }
  }

  // Extract buffer from ELF sections without assuming anything
  // about order of sections in the ELF file.
  template<typename buf_type>
  void
  initialize_buf(patcher::buf_type type, std::map<uint32_t, std::pair<uint32_t, buf_type>>& map)
  {
    for (const auto& sec : m_elfio.sections) {
      auto name = sec->get_name();
      auto sec_index = sec->get_index();
      buf_type buf;
      // Instruction buffer is in .ctrltext.* section.
      if (name.find(patcher::section_name_to_string(type)) == std::string::npos)
        continue;
      
      uint32_t index = get_section_name_index(name);
      buf.append_section_data(sec.get());
      map.emplace(std::make_pair(index, std::make_pair(sec_index, buf)));
    }
  }

#if 0
  // Extract instruction buffer from ELF sections without assuming anything
  // about order of sections in the ELF file.
  void
  initialize_instr_buf()
  {
    for (const auto& sec : m_elfio.sections) {
      auto name = sec->get_name();
      instr_buf instrbuf;
      // Instruction buffer is in .ctrltext.* section.
      if (name.find(patcher::section_name_to_string(patcher::buf_type::ctrltext)) == std::string::npos)
        continue;
      
      uint32_t index = get_section_name_index(name);
      instrbuf.append_section_data(sec.get());
      m_instr_buf_map.emplace({index, instrbuf});
    }
  }

  // Extract control-packet buffer from ELF sections without assuming anything
  // about order of sections in the ELF file.
  void
  initialize_ctrl_packet(const ELFIO::elfio& elf)
  {
    for (const auto& sec : elf.sections) {
      auto name = sec->get_name();
      control_packet ctrlpacket;
      // Instruction buffer is in .ctrldata.* section.
      if (name.find(patcher::section_name_to_string(patcher::buf_type::ctrldata)) == std::string::npos)
        continue;

      uint32_t index = get_section_name_index(name);
      ctrlpacket.append_section_data(sec.get());
      m_ctrl_packet_map.emplace({index, ctrlpacket});
    }
  }
#endif

  // Extract preempt_save buffer from ELF sections
  // return true if section exist
  bool
  initialize_save_buf(buf& save_buf)
  {
    for (const auto& sec : m_elfio.sections) {
      auto name = sec->get_name();
      if (name.find(patcher::section_name_to_string(patcher::buf_type::preempt_save)) == std::string::npos)
        continue;

      save_buf.append_section_data(sec.get());
      m_save_buf_sec_idx = sec->get_index();
      return true;
    }
    return false;
  }

  // Extract preempt_restore buffer from ELF sections
  // return true if section exist
  bool
  initialize_restore_buf(buf& restore_buf)
  {
    for (const auto& sec : m_elfio.sections) {
      auto name = sec->get_name();
      if (name.find(patcher::section_name_to_string(patcher::buf_type::preempt_restore)) == std::string::npos)
        continue;

      restore_buf.append_section_data(sec.get());
      m_restore_buf_sec_idx = sec->get_index();
      return true;
    }

    return false;
  }

#if 0
  // Extract pdi from ELF sections
  void
  initialize_pdi_buf(const ELFIO::elfio& elf)
  {
    for (const auto& sec : elf.sections) {
      auto name = sec->get_name();
      buf pdi_buf;
      if (name.find(patcher::section_name_to_string(patcher::buf_type::pdi)) == std::string::npos)
        continue;

      uint32_t index = get_section_name_index(name);
      pdi_buf.append_section_data(sec.get());
      m_pdi_buf_map.emplace({index, pdi_buf});
    }
  }
#endif

  // Extract control code from ELF sections without assuming anything
  // about order of sections in the ELF file.  Build helper data
  // structures that manages the control code data for each column and
  // page, then create ctrlcode objects from the data.
  std::vector<ctrlcode>
  initialize_column_ctrlcode()
  {
    // Elf sections for a single page
    struct column_page
    {
      ELFIO::section* ctrltext = nullptr;
      ELFIO::section* ctrldata = nullptr;
    };

    // Elf sections for a single column, the column control code is
    // divided into pages of some architecture defined size.
    struct column_sections
    {
      using page_index = uint32_t;
      std::map<page_index, column_page> pages;
    };

    // Elf ctrl code for a partition spanning multiple columns, where
    // each column has its own control code.  For architectures where
    // a partition is not divided into columns, there will be just one
    // entry in the associative map.
    // col -> [page -> [ctrltext, ctrldata]]
    using column_index = uint32_t;
    std::map<column_index, column_sections> col_secs;

    // Iterate sections in elf, collect ctrltext and ctrldata
    // per column and page
    for (const auto& sec : m_elfio.sections) {
      auto name = sec->get_name();
      if (name.find(patcher::section_name_to_string(patcher::buf_type::ctrltext)) != std::string::npos) {
        auto [col, page] = get_column_and_page(sec->get_name());
        col_secs[col].pages[page].ctrltext = sec.get();
      }
      else if (name.find(patcher::section_name_to_string(patcher::buf_type::ctrldata)) != std::string::npos) {
        auto [col, page] = get_column_and_page(sec->get_name());
        col_secs[col].pages[page].ctrldata = sec.get();
      }
    }

    // Create column control code from the collected data
    // If page requirement, then pad to page size for page
    // of a column so that embedded processor can load a page
    // at a time.
    std::vector<ctrlcode> ctrlcodes;
    ctrlcodes.resize(col_secs.size());

    for (auto& [col, col_sec] : col_secs) {
      for (auto& [page, page_sec] : col_sec.pages) {
        if (page_sec.ctrltext)
          ctrlcodes[col].append_section_data(page_sec.ctrltext);

        if (page_sec.ctrldata)
          ctrlcodes[col].append_section_data(page_sec.ctrldata);

        ctrlcodes[col].pad_to_page(page);
      }
    }

    return ctrlcodes;
  }

  std::pair<size_t, patcher::buf_type>
  determine_section_type(const std::string& section_name)
  {
    if (section_name.find(patcher::section_name_to_string(patcher::buf_type::ctrltext)) != std::string::npos) {
      auto index = get_section_name_index(section_name);
      if (index >= m_instr_buf_map.size())
        throw std::runtime_error("Invalid section passed, section info is not cached\n");
      return { m_instr_buf_map[index].second.size(), patcher::buf_type::ctrltext};
    }

    else if (!m_ctrl_packet_map.empty() &&
             section_name.find(patcher::section_name_to_string(patcher::buf_type::ctrldata)) != std::string::npos) {
      auto index = get_section_name_index(section_name);
      if (index >= m_ctrl_packet_map.size())
        throw std::runtime_error("Invalid section passed, section info is not cached\n");
      return { m_ctrl_packet_map[index].second.size(), patcher::buf_type::ctrldata};
    }

    else if (m_save_buf_exist && (section_name == patcher::section_name_to_string(patcher::buf_type::preempt_save)))
      return { m_save_buf.size(), patcher::buf_type::preempt_save };

    else if (m_restore_buf_exist && (section_name == patcher::section_name_to_string(patcher::buf_type::preempt_restore)))
      return { m_restore_buf.size(), patcher::buf_type::preempt_restore };

    else if (!m_pdi_buf_map.empty() &&
             section_name.find(patcher::section_name_to_string(patcher::buf_type::pdi)) != std::string::npos) {
      auto index = get_section_name_index(section_name);
      if (index >= m_pdi_buf_map.size())
        throw std::runtime_error("Invalid section passed, section info is not cached\n");
      return { m_pdi_buf_map[index].second.size(), patcher::buf_type::pdi };
    }

    else
      throw std::runtime_error("Invalid section name " + section_name);
  }

  std::map<std::string, patcher>
  initialize_arg_patchers()
  {
    auto dynsym = m_elfio.sections[".dynsym"];
    auto dynstr = m_elfio.sections[".dynstr"];
    auto dynsec = m_elfio.sections[".rela.dyn"];

    if (!dynsym || !dynstr || !dynsec)
      return {};

    std::ofstream ofs("arg_patchers.txt");
    if (!ofs.is_open())
      throw std::runtime_error("Failed to open file arg_patchers.txt for writing");

    std::map<std::string, patcher> arg2patchers;
    auto name = dynsec->get_name();

    // Iterate over all relocations and construct a patcher for each
    // relocation that refers to a symbol in the .dynsym section.
    auto begin = reinterpret_cast<const ELFIO::Elf32_Rela*>(dynsec->get_data());
    auto end = begin + dynsec->get_size() / sizeof(const ELFIO::Elf32_Rela);
    for (auto rela = begin; rela != end; ++rela) {
      auto symidx = ELFIO::get_sym_and_type<ELFIO::Elf32_Rela>::get_r_sym(rela->r_info);

      auto dynsym_offset = symidx * sizeof(ELFIO::Elf32_Sym);
      if (dynsym_offset >= dynsym->get_size())
        throw std::runtime_error("Invalid symbol index " + std::to_string(symidx));
      auto sym = reinterpret_cast<const ELFIO::Elf32_Sym*>(dynsym->get_data() + dynsym_offset);

      auto dynstr_offset = sym->st_name;
      if (dynstr_offset >= dynstr->get_size())
        throw std::runtime_error("Invalid symbol name offset " + std::to_string(dynstr_offset));
      auto symname = dynstr->get_data() + dynstr_offset;

      if (!m_scratch_pad_mem_size && (strcmp(symname, Scratch_Pad_Mem_Symbol) == 0)) {
        m_scratch_pad_mem_size = static_cast<size_t>(sym->st_size);
      }

      // Get control code section referenced by the symbol, col, and page
      auto section = m_elfio.sections[sym->st_shndx];
      if (!section)
        throw std::runtime_error("Invalid section index " + std::to_string(sym->st_shndx));

      auto offset = rela->r_offset;
      auto [sec_size, buf_type] = determine_section_type(section->get_name());
      auto sec_index = section->get_index();

      if (offset >= sec_size)
        throw std::runtime_error("Invalid offset " + std::to_string(offset));

      uint32_t add_end_higher_28bit = (rela->r_addend & addend_mask) >> addend_shift;
      std::string argnm{ symname, symname + std::min(strlen(symname), dynstr->get_size()) };

      auto patch_scheme = static_cast<patcher::symbol_type>(rela->r_addend & schema_mask);

      patcher::patch_info pi = patch_scheme == patcher::symbol_type::scalar_32bit_kind ?
                               // st_size is is encoded using register value mask for scaler_32
                               // for other pacthing scheme it is encoded using size of dma
                               patcher::patch_info{ offset, add_end_higher_28bit, static_cast<uint32_t>(sym->st_size) } :
                               patcher::patch_info{ offset, add_end_higher_28bit, 0 };

      std::string key_string = generate_key_string(argnm, buf_type, sec_index);

      if (auto search = arg2patchers.find(key_string); search != arg2patchers.end())
        search->second.m_ctrlcode_patchinfo.emplace_back(pi);
      else {
        arg2patchers.emplace(std::move(key_string), patcher{ patch_scheme, {pi}, buf_type});
      }

      ofs << "patch_scheme: " << static_cast<int>(patch_scheme) << " buf_type : " << static_cast<int>(buf_type) << " \t";
      ofs << "key_string: " << key_string << " offset: " << offset << " add_end_higher_28bit: " << add_end_higher_28bit << " size: " << sym->st_size << std::endl;

    }
    return arg2patchers;
  }

  std::map<std::string, patcher>
  initialize_arg_patchers(const std::vector<ctrlcode>& ctrlcodes)
  {
    auto dynsym = m_elfio.sections[".dynsym"];
    auto dynstr = m_elfio.sections[".dynstr"];

    std::map<std::string, patcher> arg2patcher;

    for (const auto& sec : m_elfio.sections) {
      auto name = sec->get_name();
      if (name.find(".rela.dyn") == std::string::npos)
        continue;

      // Iterate over all relocations and construct a patcher for each
      // relocation that refers to a symbol in the .dynsym section.
      auto begin = reinterpret_cast<const ELFIO::Elf32_Rela*>(sec->get_data());
      auto end = begin + sec->get_size() / sizeof(const ELFIO::Elf32_Rela);
      for (auto rela = begin; rela != end; ++rela) {
        auto symidx = ELFIO::get_sym_and_type<ELFIO::Elf32_Rela>::get_r_sym(rela->r_info);

        auto dynsym_offset = symidx * sizeof(ELFIO::Elf32_Sym);
        if (dynsym_offset >= dynsym->get_size())
          throw std::runtime_error("Invalid symbol index " + std::to_string(symidx));
        auto sym = reinterpret_cast<const ELFIO::Elf32_Sym*>(dynsym->get_data() + dynsym_offset);

        auto dynstr_offset = sym->st_name;
        if (dynstr_offset >= dynstr->get_size())
          throw std::runtime_error("Invalid symbol name offset " + std::to_string(dynstr_offset));
        auto symname = dynstr->get_data() + dynstr_offset;

        // Get control code section referenced by the symbol, col, and page
        auto ctrl_sec = m_elfio.sections[sym->st_shndx];
        if (!ctrl_sec)
          throw std::runtime_error("Invalid section index " + std::to_string(sym->st_shndx));
        auto [col, page] = get_column_and_page(ctrl_sec->get_name());

        auto column_ctrlcode_size = ctrlcodes.at(col).size();
        auto column_ctrlcode_offset = page * column_page_size + rela->r_offset + 16; // NOLINT magic number 16??
        if (column_ctrlcode_offset >= column_ctrlcode_size)
          throw std::runtime_error("Invalid ctrlcode offset " + std::to_string(column_ctrlcode_offset));

        // The control code for all columns will be represented as one
        // contiguous buffer object.  The patcher will need to know
        // the offset into the buffer object for the particular column
        // and page being patched.  Past first [0, col) columns plus
        // page offset within column and the relocation offset within
        // the page.
        uint64_t ctrlcode_offset = 0;
        for (uint32_t i = 0; i < col; ++i)
          ctrlcode_offset += ctrlcodes.at(i).size();
        ctrlcode_offset += column_ctrlcode_offset;

        // Construct the patcher for the argument with the symbol name
        std::string argnm{ symname, symname + std::min(strlen(symname), dynstr->get_size()) };
        patcher::buf_type buf_type = patcher::buf_type::ctrltext;

        auto symbol_type = static_cast<patcher::symbol_type>(rela->r_addend);
        arg2patcher.emplace(std::move(generate_key_string(argnm, buf_type, UINT32_MAX)), patcher{ symbol_type, {{ctrlcode_offset, 0}}, buf_type});
      }
    }

    return arg2patcher;
  }

  bool
  patch(uint8_t* base, const std::string& argnm, size_t index, uint64_t patch,
        patcher::buf_type type, uint32_t sec_index) override
  {
    std::ofstream out_file("patch_info.txt", std::ios_base::app);
    if (!out_file.is_open()) {
	    throw std::runtime_error("failed to open patch file\n");
    }

    uint32_t* p_p = reinterpret_cast<uint32_t*>(base);

    std::cout << "rahul -- inside " << __func__ << " patching - " << patch << " with argnm - " << argnm << " with index - " << index << " base - " << std::hex << p_p <<std::endl;

    const std::string key_string = generate_key_string(argnm, type, sec_index);
    auto it = m_arg2patcher.find(key_string);
    auto not_found_use_argument_name = (it == m_arg2patcher.end());
    if (not_found_use_argument_name) {// Search using index
      auto index_string = std::to_string(index);
      const std::string key_index_string = generate_key_string(index_string, type, sec_index);
      it = m_arg2patcher.find(key_index_string);
      if (it == m_arg2patcher.end())
        return false;
    }

    it->second.patch(base, patch);
    if (xrt_core::config::get_xrt_debug()) {
      if (not_found_use_argument_name) {
        std::stringstream ss;
        ss << "Patched " << patcher::section_name_to_string(type) << " using argument index " << index << " with value " << std::hex << patch << " at base - " << std::hex << p_p << std::endl;
        xrt_core::message::send( xrt_core::message::severity_level::debug, "xrt_module", ss.str());
	out_file << ss.str() << std::endl;
      }
      else {
        std::stringstream ss;
        ss << "Patched " << patcher::section_name_to_string(type) << " using argument name " << argnm << " with value " << std::hex << patch;
        xrt_core::message::send( xrt_core::message::severity_level::debug, "xrt_module", ss.str());
	out_file << ss.str() << std::endl;
      }
    }
    return true;
  }

  [[nodiscard]] uint8_t
  get_os_abi() const override
  {
    return m_os_abi;
  }

  ert_cmd_opcode
  get_ert_opcode() const override
  {
    if (m_os_abi == Elf_Amd_Aie2ps)
      return ERT_START_DPU;

    if (m_os_abi != Elf_Amd_Aie2p && m_os_abi != Elf_Amd_Aie2p_config)
      throw std::runtime_error("ELF os_abi Not supported");

    if (!m_pdi_buf_map.empty())
      return ERT_START_NPU_PDI_IN_ELF;

    if (m_save_buf_exist && m_restore_buf_exist)
      return ERT_START_NPU_PREEMPT;

    return ERT_START_NPU;
  }

public:
  explicit module_elf(xrt::elf elf)
    : module_impl{ elf.get_cfg_uuid() }
    , m_elfio(xrt_core::elf_int::get_elfio(elf))
    , m_os_abi(m_elfio.get_os_abi())
  {
    if (m_os_abi == Elf_Amd_Aie2ps) {
      m_ctrlcodes = initialize_column_ctrlcode();
      m_arg2patcher = initialize_arg_patchers(m_ctrlcodes);
    }
    else if (m_os_abi == Elf_Amd_Aie2p) {
      initialize_buf(patcher::buf_type::ctrltext, m_instr_buf_map);
      initialize_buf(patcher::buf_type::ctrldata, m_ctrl_packet_map);

      m_save_buf_exist = initialize_save_buf(m_save_buf);
      m_restore_buf_exist = initialize_restore_buf(m_restore_buf);
      if (m_save_buf_exist != m_restore_buf_exist)
        throw std::runtime_error{ "Invalid elf because preempt save and restore is not paired" };

      initialize_buf(patcher::buf_type::pdi, m_pdi_buf_map);
      m_arg2patcher = initialize_arg_patchers();
    }
    else if (m_os_abi == Elf_Amd_Aie2p_config) {
      initialize_partition_size();
      initialize_kernel_signature();
      initialize_buf(patcher::buf_type::ctrltext, m_instr_buf_map);
      initialize_buf(patcher::buf_type::ctrldata, m_ctrl_packet_map);

      m_save_buf_exist = initialize_save_buf(m_save_buf);
      m_restore_buf_exist = initialize_restore_buf(m_restore_buf);
      if (m_save_buf_exist != m_restore_buf_exist)
        throw std::runtime_error{ "Invalid elf because preempt save and restore is not paired" };

      initialize_buf(patcher::buf_type::pdi, m_pdi_buf_map);
      m_arg2patcher = initialize_arg_patchers();
    }
  }

  [[nodiscard]] const std::vector<ctrlcode>&
  get_data() const override
  {
    return m_ctrlcodes;
  }

  [[nodiscard]] std::pair<uint32_t, instr_buf>
  get_instr(uint32_t index) const override
  {
    if (m_instr_buf_map.empty() || index >= m_instr_buf_map.size())
      return std::make_pair(UINT32_MAX, instr_buf{});
    return m_instr_buf_map.at(index);
  }

  [[nodiscard]] std::pair<uint32_t, buf>
  get_preempt_save() const override
  {
    return {m_save_buf_sec_idx, m_save_buf};
  }

  [[nodiscard]] std::pair<uint32_t, buf>
  get_preempt_restore() const override
  {
      return {m_restore_buf_sec_idx, m_restore_buf};
  }

  [[nodiscard]] std::pair<uint32_t, buf>
  get_pdi(uint32_t index) const override
  {
    if (m_pdi_buf_map.empty() || index >= m_pdi_buf_map.size())
      return std::make_pair(UINT32_MAX, buf{});
    return m_pdi_buf_map.at(index);
  }

  [[nodiscard]] virtual size_t
  get_scratch_pad_mem_size() const override
  {
      return m_scratch_pad_mem_size;
  }

  [[nodiscard]] std::pair<uint32_t, control_packet>
  get_ctrlpkt(uint32_t index) const override
  {
    if (m_ctrl_packet_map.empty() || index >= m_ctrl_packet_map.size())
      return std::make_pair(UINT32_MAX, control_packet{});
    return m_ctrl_packet_map.at(index);
  }

  [[nodiscard]] size_t
  number_of_arg_patchers() const override
  {
    return m_arg2patcher.size();
  }

  [[nodiscard]] virtual uint32_t
  get_partition_size() const override
  {
    if (m_partition_size == UINT32_MAX)
      throw std::runtime_error("No partition info available, wrong ELF passed\n");

    return m_partition_size;
  }

  [[nodiscard]] virtual std::string
  get_kernel_signature() const
  {
    if (m_kernel_signature.empty())
      throw std::runtime_error("No kernel signature available, wrong ELF passed\n");
  
    return m_kernel_signature;
  }
};

// class module_userptr - Opaque userptr provided by application
class module_userptr : public module_impl
{
  std::vector<ctrlcode> m_ctrlcode;
  instr_buf m_instr_buf;
  control_packet m_ctrl_pkt;

  // Create a ctrlcode object from the userptr.
  static std::vector<ctrlcode>
  initialize_ctrlcode(const char* userptr, size_t sz)
  {
    std::vector<ctrlcode> ctrlcodes;
    ctrlcodes.resize(1);
    ctrlcodes[0].append_section_data(reinterpret_cast<const uint8_t*>(userptr), sz);
    return ctrlcodes;
  }

public:
  module_userptr(const char* userptr, size_t sz, const xrt::uuid& uuid)
    : module_impl{ uuid }
    , m_ctrlcode{ initialize_ctrlcode(userptr, sz) }
  {}

  module_userptr(const void* userptr, size_t sz, const xrt::uuid& uuid)
    : module_userptr(static_cast<const char*>(userptr), sz, uuid)
  {}

  [[nodiscard]] const std::vector<ctrlcode>&
  get_data() const override
  {
    return m_ctrlcode;
  }

  [[nodiscard]] std::pair<uint32_t, instr_buf>
  get_instr(uint32_t /*index*/) const override
  {
    return {0, m_instr_buf};
  }

  [[nodiscard]] std::pair<uint32_t, control_packet>
  get_ctrlpkt(uint32_t /*index*/) const override
  {
    return {0, m_ctrl_pkt};
  }
};

// class module_sram - Create an hwct specific (sram) module from parent
//
// Allocate a buffer object to hold the ctrlcodes for each column created
// by parent module.  The ctrlcodes are concatenated into a single buffer
// where buffer object address of offset for each column.
class module_sram : public module_impl
{
  std::shared_ptr<module_impl> m_parent;
  xrt::hw_context m_hwctx;
  // New ELFs have multiple ctrl sections
  // we need index to identify which ctrl section to pick from parent module
  uint32_t m_index;

  // The instruction buffer object contains the ctrlcodes for each
  // column.  The ctrlcodes are concatenated into a single buffer
  // padded at page size specific to hardware.
  xrt::bo m_buffer;
  xrt::bo m_instr_bo;
  xrt::bo m_ctrlpkt_bo;
  xrt::bo m_scratch_pad_mem;
  xrt::bo m_preempt_save_bo;
  xrt::bo m_preempt_restore_bo;
  xrt::bo m_pdi_bo;

  uint32_t m_instr_sec_idx;
  uint32_t m_ctrlpkt_sec_idx;

  // Column bo address is the address of the ctrlcode for each column
  // in the (sram) buffer object.  The first ctrlcode is at the base
  // address (m_buffer.address()) of the buffer object.  The addresses
  // are used in ert_dpu_data payload to identify the ctrlcode for
  // each column.
  std::vector<std::pair<uint64_t, uint64_t>> m_column_bo_address;

  // Arguments patched in the ctrlcode buffer object
  // Must match number of argument patchers in parent module
  std::set<std::string> m_patched_args;

  // Dirty bit to indicate that patching was done prior to last
  // buffer sync to device.
  bool m_dirty{ false };

  union debug_flag_union {
    struct debug_mode_struct {
      uint32_t dump_control_codes     : 1;
      uint32_t dump_control_packet    : 1;
      uint32_t dump_preemption_codes  : 1;
      uint32_t reserved : 29;
    } debug_flags;
    uint32_t all;
  }m_debug_mode = {};
  uint32_t m_id {0}; //TODO: it needs come from the elf file

  bool
  inline is_dump_control_codes() const {
    return m_debug_mode.debug_flags.dump_control_codes != 0;
  }

  bool
  inline is_dump_control_packet() const {
    return m_debug_mode.debug_flags.dump_control_packet != 0;
  }

  bool
  inline is_dump_preemption_codes() {
    return m_debug_mode.debug_flags.dump_preemption_codes != 0;
  }

  uint32_t get_id() const {
    return m_id;
  }

  // For separated multi-column control code, compute the ctrlcode
  // buffer object address of each column (used in ert_dpu_data).
  void
  fill_column_bo_address(const std::vector<ctrlcode>& ctrlcodes)
  {
    m_column_bo_address.clear();
    auto base_addr = m_buffer.address();
    for (const auto& ctrlcode : ctrlcodes) {
      m_column_bo_address.push_back({ base_addr, ctrlcode.size() }); // NOLINT
      base_addr += ctrlcode.size();
    }
  }

  void
  fill_bo_addresses()
  {
    m_column_bo_address.clear();
    m_column_bo_address.push_back({ m_instr_bo.address(), m_instr_bo.size() }); // NOLINT
  }

  // Fill the instruction buffer object with the ctrlcodes for each
  // column and sync the buffer to device.
  void
  fill_instruction_buffer(xrt::bo& bo, const std::vector<ctrlcode>& ctrlcodes)
  {
    auto ptr = bo.map<char*>();
    for (const auto& ctrlcode : ctrlcodes) {
      std::memcpy(ptr, ctrlcode.data(), ctrlcode.size());
      ptr += ctrlcode.size();
    }
    bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  }

  void
  fill_bo_with_data(xrt::bo& bo, const buf& buf)
  {
    auto ptr = bo.map<char*>();
    std::memcpy(ptr, buf.data(), buf.size());
    bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  }

  void
  fill_ctrlpkt_buf(xrt::bo& bo, const control_packet& ctrlpktbuf)
  {
    auto ptr = bo.map<char*>();
    std::memcpy(ptr, ctrlpktbuf.data(), ctrlpktbuf.size());
    bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  }

  void
  create_instr_buf(const module_impl* parent)
  {
    XRT_DEBUGF("-> module_sram::create_instr_buf()\n");
    instr_buf data;
    std::tie(m_instr_sec_idx, data) = parent->get_instr(m_index);
    size_t sz = data.size();
    if (sz == 0)
      throw std::runtime_error("Invalid instruction buffer size");

    // create bo combined size of all ctrlcodes
    m_instr_bo = xrt::bo{ m_hwctx, sz, xrt::bo::flags::cacheable, 1 /* fix me */ };

    // copy instruction into bo
    fill_bo_with_data(m_instr_bo, data);

    if (is_dump_control_codes()) {
      std::string dump_file_name = "ctr_codes_pre_patch" + std::to_string(get_id()) + ".bin";
      std::cout << "[rahul] : instr bo addr - " << std::hex << m_instr_bo.address() << std::endl;
      dump_bo(m_instr_bo, dump_file_name);

      std::stringstream ss;
      ss << "dumped file " << dump_file_name << " ctr_codes size: " << std::to_string(sz);
      xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());
    }

    auto [save_sec_idx, preempt_save_data] = parent->get_preempt_save();
    auto preempt_save_data_size = preempt_save_data.size();

    auto [restore_sec_idx, preempt_restore_data] = parent->get_preempt_restore();
    auto preempt_restore_data_size = preempt_restore_data.size();

    if ((preempt_save_data_size > 0) && (preempt_restore_data_size > 0)) {
      m_preempt_save_bo = xrt::bo{ m_hwctx, preempt_save_data_size, xrt::bo::flags::cacheable, 1 /* fix me */ };
      fill_bo_with_data(m_preempt_save_bo, preempt_save_data);

      m_preempt_restore_bo = xrt::bo{ m_hwctx, preempt_restore_data_size, xrt::bo::flags::cacheable, 1 /* fix me */ };
      fill_bo_with_data(m_preempt_restore_bo, preempt_restore_data);

      if (is_dump_preemption_codes()) {
        std::string dump_file_name = "preemption_save_pre_patch" + std::to_string(get_id()) + ".bin";
        dump_bo(m_preempt_save_bo, dump_file_name);

        std::stringstream ss;
        ss << "dumped file " << dump_file_name;
        xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());

        dump_file_name = "preemption_restore_pre_patch" + std::to_string(get_id()) + ".bin";
        dump_bo(m_preempt_restore_bo, dump_file_name);

        ss.clear();
        ss << "dumped file " << dump_file_name;
        xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());
      }
    }

    if ((preempt_save_data_size > 0) && (preempt_restore_data_size > 0)) {
      m_scratch_pad_mem = xrt::ext::bo{ m_hwctx, m_parent->get_scratch_pad_mem_size() };
      patch_instr(m_preempt_save_bo, Scratch_Pad_Mem_Symbol, 0, m_scratch_pad_mem,
                  patcher::buf_type::preempt_save, save_sec_idx);
      patch_instr(m_preempt_restore_bo, Scratch_Pad_Mem_Symbol, 0, m_scratch_pad_mem,
                  patcher::buf_type::preempt_restore, restore_sec_idx);

      if (is_dump_preemption_codes()) {
        std::stringstream ss;
        ss << "patched preemption-codes using scratch_pad_mem at address " << std::hex << m_scratch_pad_mem.address() << " size " << std::hex << m_parent->get_scratch_pad_mem_size();
        xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());
      }
    }

    auto pdi_info = parent->get_pdi(m_index);
    buf pdi_data = pdi_info.second;
    auto pdi_data_size = pdi_data.size();

    if (pdi_data_size > 0) {
      m_pdi_bo = xrt::bo{ m_hwctx, pdi_data_size, xrt::bo::flags::cacheable, 1 /* fix me */ };
      fill_bo_with_data(m_pdi_bo, pdi_data);

      if (is_dump_control_codes()) {
        std::stringstream ss;
        ss << "pdi bo address: " << std::hex << m_pdi_bo.address();
        xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());
        std::string dump_file_name = "pdi_pre_patch" + std::to_string(get_id()) + ".bin";
        dump_bo(m_pdi_bo, dump_file_name);
      }

	    // TODO: patch all pdi addresses properly
      patch_instr(m_instr_bo, pdi_symbol, 0, m_pdi_bo, patcher::buf_type::ctrltext, m_instr_sec_idx);
    }

    if (m_ctrlpkt_bo) {
      patch_instr(m_instr_bo, Control_Packet_Symbol, 0, m_ctrlpkt_bo, patcher::buf_type::ctrltext, m_instr_sec_idx);
    }
    XRT_DEBUGF("<- module_sram::create_instr_buf()\n");
  }

  void
  create_ctrlpkt_buf(const module_impl* parent)
  {
    control_packet data;
    std::tie(m_ctrlpkt_sec_idx, data) = parent->get_ctrlpkt(m_index);
    size_t sz = data.size();

    if (sz == 0) {
      XRT_DEBUGF("ctrpkt buf is empty\n");
      return;
    }

    m_ctrlpkt_bo = xrt::ext::bo{ m_hwctx, sz };

    fill_ctrlpkt_buf(m_ctrlpkt_bo, data);

    if (is_dump_control_packet()) {
      std::string dump_file_name = "ctr_packet_pre_patch" + std::to_string(get_id()) + ".bin";
      dump_bo(m_ctrlpkt_bo, dump_file_name);

      std::stringstream ss;
      ss << "dumped file " << dump_file_name;
      xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());
    }
  }

  // Create the instruction buffer object and fill it with column
  // ctrlcodes.
  void
  create_instruction_buffer(const module_impl* parent)
  {
    const auto& data = parent->get_data();

    // create bo combined size of all ctrlcodes
    size_t sz = std::accumulate(data.begin(), data.end(), static_cast<size_t>(0), [](auto acc, const auto& ctrlcode) {
      return acc + ctrlcode.size();
      });
    if (sz == 0) {
      XRT_DEBUGF("ctrcode buf is empty\n");
      return;
    }

    m_buffer = xrt::bo{ m_hwctx, sz, xrt::bo::flags::cacheable, 1 /* fix me */ };

    fill_instruction_buffer(m_buffer, data);
  }

  virtual void
  patch_instr(xrt::bo& bo_ctrlcode, const std::string& argnm, size_t index, const xrt::bo& bo,
              patcher::buf_type type, uint32_t sec_idx) override
  {
    patch_instr_value(bo_ctrlcode, argnm, index, bo.address(), type, sec_idx);
  }

  void
  patch_value(const std::string& argnm, size_t index, uint64_t value)
  {
    // HACK : index and arg num differ in new elf flow
    // this should be removed once we have proper elf
    index += 3;

    bool patched = false;
    if (m_parent->get_os_abi() == Elf_Amd_Aie2p || m_parent->get_os_abi() == Elf_Amd_Aie2p_config) {
      // patch control-packet buffer
      if (m_ctrlpkt_bo) {
        if (m_parent->patch(m_ctrlpkt_bo.map<uint8_t*>(), argnm, index, value, patcher::buf_type::ctrldata, m_ctrlpkt_sec_idx))
          patched = true;
      }

      std::cout << "--- instr bo addr - " << std::hex << m_instr_bo.address() << std::endl;
      // patch instruction buffer
      if (m_parent->patch(m_instr_bo.map<uint8_t*>(), argnm, index, value, patcher::buf_type::ctrltext, m_instr_sec_idx))
          patched = true;
    }
    else if (m_parent->patch(m_buffer.map<uint8_t*>(), argnm, index, value, patcher::buf_type::ctrltext, UINT32_MAX))
      patched = true;

    if (patched) {
      m_patched_args.insert(argnm);
      m_dirty = true;
    }
  }

  void
  patch_instr_value(xrt::bo& bo, const std::string& argnm, size_t index, uint64_t value,
                    patcher::buf_type type, uint32_t sec_index)
  {
    if (!m_parent->patch(bo.map<uint8_t*>(), argnm, index, value, type, sec_index))
      return;

    m_dirty = true;
  }

  // Check that all arguments have been patched and sync the buffer
  // to device if it is dirty.
  void
  sync_if_dirty() override
  {
    if (!m_dirty)
      return;

    auto os_abi = m_parent.get()->get_os_abi();
    if (os_abi == Elf_Amd_Aie2ps) {
      if (m_patched_args.size() != m_parent->number_of_arg_patchers()) {
        auto fmt = boost::format("ctrlcode requires %d patched arguments, but only %d are patched")
            % m_parent->number_of_arg_patchers() % m_patched_args.size();
        throw std::runtime_error{ fmt.str() };
      }
      m_buffer.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    }
    else if (os_abi == Elf_Amd_Aie2p || os_abi == Elf_Amd_Aie2p_config) {
      m_instr_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);

      if (is_dump_control_codes()) {
        std::string dump_file_name = "ctr_codes_post_patch" + std::to_string(get_id()) + ".bin";
        dump_bo(m_instr_bo, dump_file_name);

        std::stringstream ss;
        ss << "dumped file " << dump_file_name;
        xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());
      }

      if (m_ctrlpkt_bo) {
        m_ctrlpkt_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);

        if (is_dump_control_packet()) {
          std::string dump_file_name = "ctr_packet_post_patch" + std::to_string(get_id()) + ".bin";
          dump_bo(m_ctrlpkt_bo, dump_file_name);

          std::stringstream ss;
          ss << "dumped file " << dump_file_name;
          xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());
        }
      }

      if (m_preempt_save_bo && m_preempt_restore_bo) {
        m_preempt_save_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
        m_preempt_restore_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);

        if (is_dump_preemption_codes()) {
          std::string dump_file_name = "preemption_save_post_patch" + std::to_string(get_id()) + ".bin";
          dump_bo(m_preempt_save_bo, dump_file_name);

          std::stringstream ss;
          ss << "dumped file " << dump_file_name;
          xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());

          dump_file_name = "preemption_restore_post_patch" + std::to_string(get_id()) + ".bin";
          dump_bo(m_preempt_restore_bo, dump_file_name);

          ss.clear();
          ss << "dumped file " << dump_file_name;
          xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", ss.str());
        }
      }
    }

    m_dirty = false;
  }

  uint32_t*
  fill_ert_aie2p(uint32_t *payload) const
  {
     if (m_preempt_save_bo && m_preempt_restore_bo) {
       // npu preemption
       auto npu = reinterpret_cast<ert_npu_preempt_data*>(payload);
       npu->instruction_buffer = m_instr_bo.address();
       npu->instruction_buffer_size = static_cast<uint32_t>(m_instr_bo.size());
       npu->save_buffer = m_preempt_save_bo.address();
       npu->save_buffer_size = static_cast<uint32_t>(m_preempt_save_bo.size());
       npu->restore_buffer = m_preempt_restore_bo.address();
       npu->restore_buffer_size = static_cast<uint32_t>(m_preempt_restore_bo.size());
       npu->instruction_prop_count = 0; // Reserved for future use
       payload += sizeof(ert_npu_preempt_data) / sizeof(uint32_t);

       return payload;
     }

     // npu non-preemption
     auto npu = reinterpret_cast<ert_npu_data*>(payload);
     npu->instruction_buffer = m_instr_bo.address();
     npu->instruction_buffer_size = static_cast<uint32_t>(m_instr_bo.size());
     npu->instruction_prop_count = 0; // Reserved for future use
     payload += sizeof(ert_npu_data) / sizeof(uint32_t);

     return payload;
  }

  uint32_t*
  fill_ert_aie2p_config(uint32_t *payload) const
  {
    // npu preemption in elf_flow
    auto npu = reinterpret_cast<ert_npu_preempt_data*>(payload);
    npu->instruction_buffer = m_instr_bo.address();
    npu->instruction_buffer_size = static_cast<uint32_t>(m_instr_bo.size());
    npu->instruction_prop_count = 0; // Reserved for future use
    if (m_preempt_save_bo && m_preempt_restore_bo) {
      npu->save_buffer = m_preempt_save_bo.address();
      npu->save_buffer_size = static_cast<uint32_t>(m_preempt_save_bo.size());
      npu->restore_buffer = m_preempt_restore_bo.address();
      npu->restore_buffer_size = static_cast<uint32_t>(m_preempt_restore_bo.size());
    }
    payload += sizeof(ert_npu_preempt_data) / sizeof(uint32_t);
    return payload;
  }

  uint32_t*
  fill_ert_aie2ps(uint32_t *payload) const
  {
    auto ert_dpu_data_count = static_cast<uint32_t>(m_column_bo_address.size());
    // For multiple instruction buffers, the ert_dpu_data::chained has
    // the number of words remaining in the payload after the current
    // instruction buffer. The ert_dpu_data::chained of the last buffer
    // is zero.
    for (auto [addr, size] : m_column_bo_address) {
      auto dpu = reinterpret_cast<ert_dpu_data*>(payload);
      dpu->instruction_buffer = addr;
      dpu->instruction_buffer_size = static_cast<uint32_t>(size);
      dpu->chained = --ert_dpu_data_count;
      payload += sizeof(ert_dpu_data) / sizeof(uint32_t);
    }

    return payload;
  }

public:
  module_sram(std::shared_ptr<module_impl> parent, xrt::hw_context hwctx, uint32_t index = 0)
    : module_impl{ parent->get_cfg_uuid() }
    , m_parent{ std::move(parent) }
    , m_hwctx{ std::move(hwctx) }
    , m_index{ index }
  {
    if (xrt_core::config::get_xrt_debug()) {
      m_debug_mode.debug_flags.dump_control_codes = xrt_core::config::get_feature_toggle("Debug.dump_control_codes");
      m_debug_mode.debug_flags.dump_control_packet = xrt_core::config::get_feature_toggle("Debug.dump_control_packet");
      m_debug_mode.debug_flags.dump_preemption_codes = xrt_core::config::get_feature_toggle("Debug.dump_preemption_codes");
      static std::atomic<uint32_t> s_id {0};
      m_id = s_id++;
    }

    auto os_abi = m_parent.get()->get_os_abi();

    if (os_abi == Elf_Amd_Aie2p || os_abi == Elf_Amd_Aie2p_config) {
      // make sure to create control-packet buffer first because we may
      // need to patch control-packet address to instruction buffer
      create_ctrlpkt_buf(m_parent.get());
      create_instr_buf(m_parent.get());
      fill_bo_addresses();
    }
    else if (os_abi == Elf_Amd_Aie2ps) {
      create_instruction_buffer(m_parent.get());
      fill_column_bo_address(m_parent->get_data());
    }
  }

  uint32_t*
  fill_ert_dpu_data(uint32_t *payload) const override
  {
    auto os_abi = m_parent.get()->get_os_abi();

    if (os_abi == Elf_Amd_Aie2p)
      return fill_ert_aie2p(payload);
    else if (os_abi == Elf_Amd_Aie2p_config)
      return fill_ert_aie2p_config(payload);

    return fill_ert_aie2ps(payload);
  }

#if 0
  uint32_t*
  fill_ert_dpu_data_elf_flow(uint32_t *payload) const override
  {
    // npu preemption in elf_flow
    auto npu = reinterpret_cast<ert_npu_preempt_data*>(payload);
    npu->instruction_buffer = m_instr_bo.address();
    npu->instruction_buffer_size = static_cast<uint32_t>(m_instr_bo.size());
    npu->instruction_prop_count = 0; // Reserved for future use
    if (m_preempt_save_bo && m_preempt_restore_bo) {
      npu->save_buffer = m_preempt_save_bo.address();
      npu->save_buffer_size = static_cast<uint32_t>(m_preempt_save_bo.size());
      npu->restore_buffer = m_preempt_restore_bo.address();
      npu->restore_buffer_size = static_cast<uint32_t>(m_preempt_restore_bo.size());
    }
    payload += sizeof(ert_npu_preempt_data) / sizeof(uint32_t);
    return payload;
  }
#endif

  [[nodiscard]] virtual xrt::bo&
      get_scratch_pad_mem() override
  {
      return m_scratch_pad_mem;
  }

  void
  dump_scratchpad_mem()
  {
    if (m_scratch_pad_mem.size() == 0) {
      xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module",
                              "preemption scratchpad memory is not available");
      return;
    }

    // sync data from device before dumping into file
    m_scratch_pad_mem.sync(XCL_BO_SYNC_BO_FROM_DEVICE);

    std::string dump_file_name = "preemption_scratchpad_mem" + std::to_string(get_id()) + ".bin";
    dump_bo(m_scratch_pad_mem, dump_file_name);

    std::string msg {"dumped file "};
    msg.append(dump_file_name);
    xrt_core::message::send(xrt_core::message::severity_level::debug, "xrt_module", msg);
  }

  void
  patch(const std::string& argnm, size_t index, const xrt::bo& bo) override
  {
    patch_value(argnm, index, bo.address());
  }

  void
  patch(const std::string& argnm, size_t index, const void* value, size_t size) override
  {
    if (size > 8) // NOLINT
      throw std::runtime_error{ "patch_value() only supports 64-bit values or less" };
    
    auto arg_value = *static_cast<const uint64_t*>(value);
    patch_value(argnm, index, arg_value);
  }
};

} // namespace xrt

////////////////////////////////////////////////////////////////
// XRT implmentation access to internal module APIs
////////////////////////////////////////////////////////////////
namespace xrt_core::module_int {

uint32_t*
fill_ert_dpu_data(const xrt::module& module, uint32_t* payload)
{
  return module.get_handle()->fill_ert_dpu_data(payload);
}

//uint32_t*
//fill_ert_dpu_data_elf_flow(const xrt::module& module, uint32_t* payload)
//{
//  return module.get_handle()->fill_ert_dpu_data_elf_flow(payload);
//}

void
patch(const xrt::module& module, const std::string& argnm, size_t index, const xrt::bo& bo)
{
  auto module_sram = std::dynamic_pointer_cast<xrt::module_sram>(module.get_handle());
  if (!module_sram)
    throw std::runtime_error("Getting module_sram failed, wrong module object passed\n");
  module_sram->patch(argnm, index, bo);
}

void
patch(const xrt::module& module, uint8_t* ibuf, size_t* sz, const std::vector<std::pair<std::string, uint64_t>>* args,
      uint32_t idx)
{
  auto hdl = module.get_handle();
  size_t orig_sz = *sz;
  const buf* inst = nullptr;
  uint32_t patch_index = UINT32_MAX;

  if (hdl->get_os_abi() == Elf_Amd_Aie2p || Elf_Amd_Aie2p_config) {
    auto instr_info = hdl->get_instr(idx);
    inst = &(instr_info.second);
    patch_index = instr_info.first;
  }
  else if(hdl->get_os_abi() == Elf_Amd_Aie2ps) {
    const auto& instr_buf = hdl->get_data();
    if (instr_buf.size() != 1)
      throw std::runtime_error{"Patch failed: only support patching single column"};
    inst = &instr_buf[0];
  }
  else {
    throw std::runtime_error{"Patch failed: unsupported ELF ABI"};
  }

  *sz = inst->size();
  if (orig_sz == 0)
    return; // Caller is discovering real size of control code buffer.

  if (orig_sz < *sz)
    throw std::runtime_error{"Control code buffer passed in is too small"}; // Need a bigger buffer.
  std::memcpy(ibuf, inst->data(), *sz);

  size_t index = 0;
  for (auto& [arg_name, arg_addr] : *args) {
    if (!hdl->patch(ibuf, arg_name, index, arg_addr, patcher::buf_type::ctrltext, patch_index))
      throw std::runtime_error{"Failed to patch " + arg_name};
    index++;
  }
}

void
patch(const xrt::module& module, const std::string& argnm, size_t index, const void* value, size_t size)
{
  auto module_sram = std::dynamic_pointer_cast<xrt::module_sram>(module.get_handle());
  if (!module_sram)
    throw std::runtime_error("Getting module_sram failed, wrong module object passed\n");
  module_sram->patch(argnm, index, value, size);
}

void
sync(const xrt::module& module)
{
  module.get_handle()->sync_if_dirty();
}

enum ert_cmd_opcode
get_ert_opcode(const xrt::module& module)
{
  return module.get_handle()->get_ert_opcode();
}

void
dump_scratchpad_mem(const xrt::module& module)
{
  auto module_sram = std::dynamic_pointer_cast<xrt::module_sram>(module.get_handle());
  if (!module_sram)
    throw std::runtime_error("Getting module_sram failed, wrong module object passed\n");

  module_sram->dump_scratchpad_mem();
}

std::string
get_kernel_signature(const xrt::module& module)
{
  return module.get_handle()->get_demangled_kernel_signature();
}

uint32_t
get_partition_size(const xrt::module& module)
{
  return module.get_handle()->get_partition_size();
}

} // xrt_core::module_int

////////////////////////////////////////////////////////////////
// xrt_module C++ API implementation (xrt_module.h)
////////////////////////////////////////////////////////////////
namespace xrt
{
module::
module(const xrt::elf& elf)
: detail::pimpl<module_impl>{ std::make_shared<module_elf>(elf) }
{}

module::
module(void* userptr, size_t sz, const xrt::uuid& uuid)
: detail::pimpl<module_impl>{ std::make_shared<module_userptr>(userptr, sz, uuid) }
{}

module::
module(const xrt::module& parent, const xrt::hw_context& hwctx)
: detail::pimpl<module_impl>{ std::make_shared<module_sram>(parent.handle, hwctx) }
{}

xrt::uuid
module::
get_cfg_uuid() const
{
  return handle->get_cfg_uuid();
}

xrt::hw_context
module::
get_hw_context() const
{
  return handle->get_hw_context();
}

} // namespace xrt
