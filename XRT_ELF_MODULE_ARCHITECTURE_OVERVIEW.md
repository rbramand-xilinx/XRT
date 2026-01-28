# XRT ELF and Module Architecture - High-Level Overview

## Document Purpose

This document provides a high-level overview of the XRT ELF and Module architecture, focusing on the class hierarchy, dependencies, and the flow of operations from user API to runtime execution.

**Commits Covered**: `4e6abd6988fbc33b1c04446fe301aa75d8fab40` to `cf66705e5996fd964e27ffbe6755fbf21b4b72d0`

---

## Architecture Flow

### User Workflow

```
1. User creates xrt::elf
   ↓
2. User creates xrt::hw_context with elf
   ↓
3. User creates xrt::kernel with hw_context and kernel_name
   ↓
4. User creates xrt::run with kernel
   ↓
5. User sets arguments on run
   ↓
6. User executes run
```

### Internal Object Flow

```
xrt::elf (user API)
    ↓ [parsing based on platform]
    ├── elf_aie2p (AIE2P platform)
    └── elf_aie2ps (AIE2PS/AIE4 platform)
    
xrt::hw_context (user API)
    ↓ [registers elf with kernel names]
    └── stores map<kernel_name, xrt::elf>

xrt::kernel (user API)
    ↓ [retrieves elf from hw_context by kernel_name]
    └── creates xrt::module (container holding elf)
        └── module_impl (holds shared_ptr<elf_impl>)

xrt::run (user API)
    ↓ [creates platform-specific module_run]
    ├── module_run_aie2p (AIE2P execution)
    │   └── creates xrt::bo's from elf data
    │
    └── module_run_aie2ps (AIE2PS execution)
        └── creates xrt::bo's from elf data

When user sets arguments:
    └── module_run::patch() using symbol_patcher
        └── patches xrt::bo's with argument values
```

---

## Class Hierarchy and Inheritance

### ELF Classes

```
┌─────────────────────────────────────────────┐
│           xrt::elf (User API)               │
│   Public wrapper for ELF functionality      │
└──────────────────┬──────────────────────────┘
                   │ holds shared_ptr
                   │
┌──────────────────▼──────────────────────────┐
│          elf_impl (Abstract Base)           │
│   Internal implementation of xrt::elf       │
│   - Parses ELF file                         │
│   - Stores parsed data                      │
│   - Stores patching configuration           │
└──────────────────┬──────────────────────────┘
                   │ inherits
         ┌─────────┴─────────┐
         │                   │
┌────────▼────────┐  ┌───────▼────────┐
│   elf_aie2p     │  │   elf_aie2ps   │
│ (AIE2P Platform)│  │(AIE2PS/AIE4)   │
│ - Instr buffers │  │ - Ctrlcodes    │
│ - Control pkts  │  │ - Control pkts │
│ - PDI buffers   │  │ - Dump buffers │
│ - Preemption    │  │                │
└─────────────────┘  └────────────────┘
```

**Inheritance**: `elf_aie2p` and `elf_aie2ps` inherit from `elf_impl`

**Dependencies**:
- `elf_impl` uses `ELFIO` library for parsing
- `elf_impl` uses `elf_patcher` types for patching configuration
- Platform-specific classes store section data as `buf` objects

---

### Module Classes

```
┌─────────────────────────────────────────────┐
│         xrt::module (User API)              │
│   Public wrapper - backward compatibility   │
│   Container that holds elf object           │
└──────────────────┬──────────────────────────┘
                   │ holds shared_ptr
                   │
┌──────────────────▼──────────────────────────┐
│       module_impl (Base Class)              │
│   Base implementation for modules           │
│   - Holds shared_ptr<elf_impl>              │
│   - No hardware context                     │
└──────────────────┬──────────────────────────┘
                   │ inherits
                   │
┌──────────────────▼──────────────────────────┐
│       module_run (Abstract Base)            │
│   Module with hardware context              │
│   - Holds hw_context                        │
│   - Holds ctrl_code_id                      │
│   - Manages patchers                        │
│   - Tracks patched arguments                │
└──────────────────┬──────────────────────────┘
                   │ inherits
         ┌─────────┴─────────┐
         │                   │
┌────────▼────────┐  ┌───────▼────────────┐
│module_run_aie2p │  │ module_run_aie2ps  │
│(AIE2P Execution)│  │(AIE2PS/AIE4 Exec)  │
│ - Creates BOs   │  │ - Creates BOs      │
│ - Patches BOs   │  │ - Patches BOs      │
│ - Syncs to dev  │  │ - Syncs to device  │
└─────────────────┘  └────────────────────┘
```

**Inheritance**: `module_impl` ← `module_run` ← `module_run_aie2p`/`module_run_aie2ps`

**Dependencies**:
- All module classes depend on `elf_impl` (through shared_ptr)
- `module_run` depends on `xrt::hw_context` for device access
- `module_run_aie2p/ps` depend on `xrt::bo` for buffer management
- `module_run` uses `symbol_patcher` for argument patching

---

### Patching System Classes

```
┌─────────────────────────────────────────────┐
│     patcher_config (Static Config)          │
│   Shared configuration for patching         │
│   - Owned by elf_impl                       │
│   - Read-only after ELF parsing             │
│   - Contains all patch locations            │
└──────────────────┬──────────────────────────┘
                   │ contains
                   │
┌──────────────────▼──────────────────────────┐
│      patch_config (Patch Location)          │
│   Configuration for single patch point      │
│   - Buffer offset                           │
│   - Address offset                          │
│   - Mask (for scalar patches)               │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│   symbol_patcher (Runtime Patcher)          │
│   Per-instance patcher with state           │
│   - Holds const pointer to patcher_config   │
│   - Owns patch_state vector                 │
│   - Performs actual patching                │
└──────────────────┬──────────────────────────┘
                   │ owns
                   │
┌──────────────────▼──────────────────────────┐
│      patch_state (Runtime State)            │
│   State for each patch location             │
│   - Dirty flag                              │
│   - Cached original values                  │
└─────────────────────────────────────────────┘
```

**Dependency Flow**: `patcher_config` → `symbol_patcher` → `patch_state`

**Usage**:
- `elf_impl` creates and stores `patcher_config` during parsing
- `module_run` creates `symbol_patcher` on first use (lazy)
- `symbol_patcher` patches `xrt::bo` objects

---

### Configuration Structures

```
┌─────────────────────────────────────────────┐
│   module_config_aie2p (AIE2P Config)        │
│   Platform-specific configuration           │
│   - References to ELF buffer data           │
│   - Scratch pad sizes                       │
│   - Preemption info                         │
│   - Pointer to parent elf_impl              │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│   module_config_aie2ps (AIE2PS Config)      │
│   Platform-specific configuration           │
│   - References to ELF buffer data           │
│   - Control codes                           │
│   - Control packet buffers                  │
│   - Pointer to parent elf_impl              │
└─────────────────────────────────────────────┘

module_config = std::variant<module_config_aie2p, 
                             module_config_aie2ps>
```

**Usage**: Configuration structures bridge `elf_impl` and `module_run`
- Created by `elf_impl::get_module_config()`
- Used by `module_run` constructors to create BOs

---

## Class Purposes and Responsibilities

### User-Facing Classes

#### `xrt::elf` (xrt_elf.h)
**Purpose**: User API for loading and managing ELF files

**Responsibilities**:
- Load ELF from file or memory buffer
- Provide access to kernel information
- Expose metadata (UUID, platform, ABI version)

**Usage**: Created by user, passed to `xrt::hw_context`

---

#### `xrt::module` (xrt_module.h)
**Purpose**: User API for kernel module (backward compatibility container)

**Responsibilities**:
- Hold reference to ELF implementation
- Provide stable user-facing interface
- Bridge to internal module implementation

**Usage**: Created internally by `xrt::kernel`, used by `xrt::run`

---

### ELF Implementation Classes

#### `elf_impl` (elf_int.h, xrt_elf.cpp)
**Purpose**: Abstract base class for platform-independent ELF functionality

**Responsibilities**:
- Parse ELF file structure (sections, symbols, relocations)
- Build kernel information (names, arguments, instances)
- Store section data in maps (by ctrl_code_id)
- Parse and store patching configuration (`m_arg2patcher`)
- Provide virtual interface for platform-specific operations

**Key Members**:
- `m_elfio` - ELFIO parser object
- `m_platform` - Platform type (AIE2P/AIE2PS/AIE4)
- `m_arg2patcher` - Map of ctrl_code_id → (arg_name → patcher_config)
- `m_kernels` - Vector of parsed kernel information
- `m_section_to_group_map` - Section index to group mapping
- `m_kernel_name_to_id_map` - Kernel name to ctrl_code_id mapping

**Virtual Methods**:
- `get_module_config(ctrl_code_id)` - Returns platform-specific config
- `get_ctrlcode_id(kernel_name)` - Maps kernel name to ctrl_code_id
- `is_group_elf()` - Checks if ELF uses .group sections
- `get_ert_opcode()` - Returns ERT command opcode

**Usage**: Base class, never instantiated directly

---

#### `elf_aie2p` (xrt_elf.cpp)
**Purpose**: AIE2P platform-specific ELF implementation

**Responsibilities**:
- Parse AIE2P-specific sections (.ctrltext, .ctrldata, .pdi)
- Store instruction buffers, control packets, PDI buffers
- Manage preemption buffers (save/restore)
- Calculate scratch pad memory sizes
- Create `module_config_aie2p` for module_run creation

**Key Members**:
- `m_instr_buf_map` - Map of ctrl_code_id → instruction buffer
- `m_ctrlpkt_buf_map` - Map of ctrl_code_id → control packet
- `m_preempt_save_buf_map` - Map of ctrl_code_id → save buffer
- `m_preempt_restore_buf_map` - Map of ctrl_code_id → restore buffer
- `m_pdi_buf_cache` - Map of symbol_name → PDI buffer (lazy loaded)
- `m_ctrlpkt_pm_buf_map` - Map of section → preemption control packet
- `m_scratch_pad_mem_size` - Total scratch pad memory size
- `m_ctrl_scratch_pad_mem_size` - Control scratch pad size

**Platform Detection**: Selected when `elf.get_platform() == xrt::elf::platform::aie2p`

**Usage**: Created when user creates `xrt::elf` for AIE2P platform

---

#### `elf_aie2ps` (xrt_elf.cpp)
**Purpose**: AIE2PS/AIE4 platform-specific ELF implementation

**Responsibilities**:
- Parse AIE2PS/AIE4-specific sections (.ctrltext, .ctrlpkt, .dump)
- Store control codes (padded to page size)
- Store control packet buffers
- Store dump buffer for debug/trace
- Create `module_config_aie2ps` for module_run creation

**Key Members**:
- `m_ctrlcodes` - Vector of control codes (one per ctrl_code_id)
- `m_ctrlpkt_buf_map` - Map of section_name → control packet buffer
- `m_dump_buf` - Debug/trace dump buffer

**Platform Detection**: Selected when `elf.get_platform() == xrt::elf::platform::aie2ps` or `aie4`

**Usage**: Created when user creates `xrt::elf` for AIE2PS/AIE4 platform

---

### Module Implementation Classes

#### `module_impl` (xrt_module.cpp)
**Purpose**: Base implementation for module objects without hardware context

**Responsibilities**:
- Hold shared pointer to `elf_impl`
- Provide base interface for module operations
- Throw "not supported" for operations requiring hardware context

**Key Members**:
- `m_elf_impl` - Shared pointer to ELF implementation

**Usage**: Base class for modules; `xrt::kernel` creates this type initially

---

#### `module_run` (xrt_module.cpp)
**Purpose**: Abstract base for modules with hardware context (execution-ready)

**Responsibilities**:
- Hold hardware context and ctrl_code_id
- Manage runtime patchers (map of arg_name → symbol_patcher)
- Track which arguments have been patched
- Implement patching logic using symbol_patcher
- Manage dirty flag and sync operations

**Key Members**:
- `m_hwctx` - Hardware context for device access
- `m_ctrl_code_id` - Control code ID (identifies kernel + instance)
- `m_patcher_configs` - Const pointer to shared patcher configs from elf_impl
- `m_patchers` - Map of arg_name → symbol_patcher (runtime, per-instance)
- `m_patched_args` - Set of argument names that have been patched
- `m_dirty` - Flag indicating if buffer needs sync
- `m_first_patch` - Flag for first-time buffer initialization

**Key Methods**:
- `patch(arg_name, index, value)` - Patches argument into buffer
- `sync_if_dirty()` - Syncs buffer to device if dirty

**Usage**: Base class, never instantiated directly; created by `xrt::run`

---

#### `module_run_aie2p` (xrt_module.cpp)
**Purpose**: AIE2P platform-specific module for execution

**Responsibilities**:
- Store `module_config_aie2p` (references to ELF data)
- Create `xrt::bo` objects for all buffers:
  - Instruction buffer (`m_instr_bo`)
  - Control packet buffer (`m_ctrlpkt_bo`)
  - Preemption save/restore buffers
  - Scratch pad memory buffers
  - PDI buffers (lazy, on-demand)
  - Control packet preemption buffers (lazy, on-demand)
- Fill BOs with data from ELF (using config references)
- Implement `fill_ert_dpu_data()` to populate ERT command
- Implement `sync_if_dirty()` to sync all BOs to device

**Key Members**:
- `m_config` - Platform-specific configuration (references to ELF data)
- `m_instr_bo` - Instruction buffer BO
- `m_ctrlpkt_bo` - Control packet BO
- `m_preempt_save_bo` - Preemption save BO
- `m_preempt_restore_bo` - Preemption restore BO
- `m_scratch_pad_mem_bo` - Scratch pad memory BO
- `m_ctrl_scratch_pad_mem_bo` - Control scratch pad BO
- `m_pdi_bos` - Map of symbol → PDI BO (created on-demand)
- `m_ctrlpkt_pm_bos` - Map of symbol → control packet preemption BO

**Creation Flow**:
```
1. Get module_config_aie2p from elf_aie2p
2. Create module_run_aie2p with config
3. Constructor creates BOs:
   - Allocate BO with size from config
   - Map BO to CPU memory
   - Copy data from ELF buffer (via config reference)
   - Sync BO to device
4. Get patcher configs pointer from elf_impl
5. Return module_run_aie2p (as xrt::module)
```

**Usage**: Created when `xrt::run` is created for AIE2P kernel

---

#### `module_run_aie2ps` (xrt_module.cpp)
**Purpose**: AIE2PS/AIE4 platform-specific module for execution

**Responsibilities**:
- Store `module_config_aie2ps` (references to ELF data)
- Create `xrt::bo` objects for all buffers:
  - Control code buffer (`m_ctrlcode_bo`)
  - Dump buffer (`m_dump_bo`)
  - Control packet buffers (lazy, on-demand)
- Fill BOs with data from ELF (using config references)
- Implement `fill_ert_dpu_data()` to populate ERT command
- Implement `sync_if_dirty()` to sync all BOs to device

**Key Members**:
- `m_config` - Platform-specific configuration (references to ELF data)
- `m_ctrlcode_bo` - Control code buffer BO
- `m_dump_bo` - Debug/trace dump buffer BO
- `m_ctrlpkt_bos` - Map of symbol → control packet BO (created on-demand)

**Creation Flow**:
```
1. Get module_config_aie2ps from elf_aie2ps
2. Create module_run_aie2ps with config
3. Constructor creates BOs:
   - Allocate BO with size from config
   - Map BO to CPU memory
   - Copy data from ELF buffer (via config reference)
   - Sync BO to device
4. Get patcher configs pointer from elf_impl
5. Return module_run_aie2ps (as xrt::module)
```

**Usage**: Created when `xrt::run` is created for AIE2PS/AIE4 kernel

---

### Patching System Classes

#### `patcher_config` (elf_patcher.h, elf_patcher.cpp)
**Purpose**: Static patching configuration shared across module instances

**Responsibilities**:
- Store symbol type (patch scheme: 48-bit, 57-bit, 64-bit, etc.)
- Store buffer type (which buffer to patch: ctrltext, ctrldata, pdi, etc.)
- Store vector of patch locations (patch_config objects)
- Provide methods to add patch locations

**Key Members**:
- `m_symbol_type` - Type of symbol (determines patch algorithm)
- `m_buf_type` - Type of buffer to patch
- `m_patch_configs` - Vector of patch locations

**Lifecycle**:
- Created by `elf_impl` during ELF parsing (in `.rela` section processing)
- Stored in `elf_impl::m_arg2patcher` map
- Referenced by `symbol_patcher` (never copied)
- Read-only after ELF parsing completes

**Usage**: Owned by `elf_impl`, referenced by all `symbol_patcher` instances

---

#### `patch_config` (elf_patcher.h)
**Purpose**: Configuration for a single patch location

**Responsibilities**:
- Store buffer offset (where to patch in the buffer)
- Store address offset (offset from BO base address)
- Store mask (for 32-bit scalar patches)

**Key Members**:
- `offset_to_patch_buffer` - Offset in target buffer (e.g., instruction buffer offset)
- `offset_to_base_bo_addr` - Offset to add to BO address before patching
- `mask` - Bit mask for scalar patches

**Usage**: Contained in `patcher_config::m_patch_configs` vector

---

#### `symbol_patcher` (elf_patcher.h, elf_patcher.cpp)
**Purpose**: Runtime patcher for a specific argument (per-instance)

**Responsibilities**:
- Hold const pointer to shared `patcher_config`
- Own per-instance `patch_state` vector
- Implement `patch_symbol()` to patch BO with value
- Implement `patch_symbol_raw()` for raw buffer patching (shim tests)
- Track dirty state and original values per patch location

**Key Members**:
- `m_config` - Const pointer to shared patcher_config (from elf_impl)
- `m_states` - Vector of patch_state (one per patch location)

**Key Methods**:
- `patch_symbol(bo, value, first)` - Patches all locations in BO
- `patch_symbol_raw(base, value, config)` - Static method for raw buffer patching

**Lifecycle**:
- Created by `module_run::patch()` on first use (lazy)
- Stored in `module_run::m_patchers` map
- One instance per argument name
- Destroyed when `module_run` is destroyed

**Usage**: Owned by `module_run`, used to patch arguments into BOs

---

#### `patch_state` (elf_patcher.h)
**Purpose**: Runtime state for a single patch location

**Responsibilities**:
- Track whether location has been patched (dirty flag)
- Cache original buffer values for restoration if needed

**Key Members**:
- `dirty` - Boolean flag indicating if this location was patched
- `bd_data_ptrs` - Array of original buffer descriptor values

**Usage**: Contained in `symbol_patcher::m_states` vector

---

### Configuration Structure Classes

#### `module_config_aie2p` (elf_int.h)
**Purpose**: AIE2P platform-specific configuration structure

**Responsibilities**:
- Provide references to ELF buffer data
- Provide buffer sizes
- Provide patching metadata
- Provide pointer to parent elf_impl (for lazy PDI loading)

**Key Members** (all references or values from elf_aie2p):
- `instr_data` - Reference to instruction buffer
- `ctrl_packet_data` - Reference to control packet buffer
- `preempt_save_data` - Reference to preemption save buffer
- `preempt_restore_data` - Reference to preemption restore buffer
- `scratch_pad_mem_size` - Size of scratch pad memory
- `ctrl_scratch_pad_mem_size` - Size of control scratch pad
- `patch_pdi_symbols` - Set of PDI symbols that need patching
- `ctrlpkt_pm_dynsyms` - Set of control packet preemption symbols
- `ctrlpkt_pm_bufs` - Map of control packet preemption buffers
- `has_preemption` - Flag indicating preemption support
- `elf_parent` - Pointer to parent elf_impl

**Usage**: Created by `elf_aie2p::get_module_config()`, used by `module_run_aie2p` constructor

---

#### `module_config_aie2ps` (elf_int.h)
**Purpose**: AIE2PS/AIE4 platform-specific configuration structure

**Responsibilities**:
- Provide references to ELF buffer data
- Provide pointer to parent elf_impl

**Key Members** (all references from elf_aie2ps):
- `ctrlcodes` - Reference to control codes vector
- `ctrlpkt_bufs` - Reference to control packet buffers map
- `dump_buf` - Reference to dump buffer
- `elf_parent` - Pointer to parent elf_impl

**Usage**: Created by `elf_aie2ps::get_module_config()`, used by `module_run_aie2ps` constructor

---

## Detailed Operation Flows

### 1. ELF Creation and Parsing

```
User creates xrt::elf("kernel.elf")
    │
    ├─> Load ELFIO from file
    │
    ├─> Read ELF header to determine platform
    │   └─> OS_ABI byte indicates AIE2P vs AIE2PS vs AIE4
    │
    ├─> Create platform-specific elf_impl:
    │   │
    │   ├─> AIE2P: new elf_aie2p(elfio)
    │   │   │
    │   │   ├─> Call elf_impl::parse_group_sections()
    │   │   │   └─> Builds m_section_to_group_map
    │   │   │   └─> Builds m_kernel_name_to_id_map
    │   │   │
    │   │   ├─> Parse platform-specific sections:
    │   │   │   ├─> .ctrltext.* → m_instr_buf_map
    │   │   │   ├─> .ctrldata.* → m_ctrlpkt_buf_map
    │   │   │   ├─> .preempt_save.* → m_preempt_save_buf_map
    │   │   │   ├─> .preempt_restore.* → m_preempt_restore_buf_map
    │   │   │   ├─> .pdi.* → (symbols recorded, lazy loaded)
    │   │   │   ├─> .ctrlpkt.pm.* → m_ctrlpkt_pm_buf_map
    │   │   │   └─> .pad.* → calculate scratch pad sizes
    │   │   │
    │   │   └─> Parse .rela sections (patching info):
    │   │       └─> For each relocation:
    │   │           ├─> Extract symbol name (argument name)
    │   │           ├─> Extract symbol type (patch scheme)
    │   │           ├─> Extract buffer type (from section name)
    │   │           ├─> Extract patch location (rela offset)
    │   │           ├─> Extract address offset (rela addend)
    │   │           └─> Store in m_arg2patcher[ctrl_code_id][arg_name]
    │   │
    │   └─> AIE2PS: new elf_aie2ps(elfio)
    │       │
    │       ├─> Call elf_impl::parse_group_sections()
    │       │
    │       ├─> Parse platform-specific sections:
    │       │   ├─> .ctrltext.* → m_ctrlcodes (padded to 8KB)
    │       │   ├─> .ctrlpkt.* → m_ctrlpkt_buf_map
    │       │   └─> .dump → m_dump_buf
    │       │
    │       └─> Parse .rela sections (same as AIE2P)
    │
    └─> Return xrt::elf (holding shared_ptr<elf_impl>)
```

**Key Insight**: All static patching configuration is built during ELF parsing and stored in `m_arg2patcher`. This is shared across all module instances and is read-only after parsing.

---

### 2. Hardware Context and Kernel Creation

```
User creates xrt::hw_context(device, xclbin, elf)
    │
    ├─> Register ELF with hw_context:
    │   └─> For each kernel in elf.get_kernels():
    │       └─> m_elf_map[kernel.name] = elf
    │
    └─> Return hw_context

User creates xrt::kernel(hwctx, "kernel_name")
    │
    ├─> Retrieve ELF from hw_context:
    │   └─> elf = hwctx.m_elf_map["kernel_name"]
    │
    ├─> Create module container:
    │   └─> module_impl(elf)
    │       └─> Stores m_elf_impl = elf.get_handle()
    │
    └─> Return xrt::kernel (holding xrt::module)
```

**Key Insight**: `xrt::module` at this stage is just a lightweight container holding a reference to `elf_impl`. No BOs are created yet. This is for backward compatibility.

---

### 3. Run Object Creation (module_run creation)

```
User creates xrt::run(kernel)
    │
    ├─> Get module from kernel
    │
    ├─> Get ELF from module:
    │   └─> elf_impl = module.get_elf_handle()
    │
    ├─> Get ctrl_code_id from kernel name:
    │   └─> ctrl_code_id = elf_impl->get_ctrlcode_id(kernel_name)
    │
    ├─> Pre-create control packet BO (if needed):
    │   └─> ctrlpkt_bo = xrt::bo(hwctx, size, ...)
    │
    ├─> Create module_run:
    │   └─> module_int::create_module_run(elf, hwctx, ctrl_code_id, ctrlpkt_bo)
    │       │
    │       ├─> Get platform-specific config:
    │       │   └─> module_config = elf_impl->get_module_config(ctrl_code_id)
    │       │       │
    │       │       ├─> AIE2P returns module_config_aie2p variant:
    │       │       │   └─> module_config_aie2p {
    │       │       │       .instr_data = m_instr_buf_map[ctrl_code_id],
    │       │       │       .ctrl_packet_data = m_ctrlpkt_buf_map[ctrl_code_id],
    │       │       │       .preempt_save_data = m_preempt_save_buf_map[ctrl_code_id],
    │       │       │       .preempt_restore_data = m_preempt_restore_buf_map[ctrl_code_id],
    │       │       │       .scratch_pad_mem_size = m_scratch_pad_mem_size,
    │       │       │       .ctrl_scratch_pad_mem_size = m_ctrl_scratch_pad_mem_size,
    │       │       │       // ... other fields ...
    │       │       │       .elf_parent = this
    │       │       │   }
    │       │       │
    │       │       └─> AIE2PS returns module_config_aie2ps variant:
    │       │           └─> module_config_aie2ps {
    │       │               .ctrlcodes = m_ctrlcodes,
    │       │               .ctrlpkt_bufs = m_ctrlpkt_buf_map,
    │       │               .dump_buf = m_dump_buf,
    │       │               .elf_parent = this
    │       │           }
    │       │
    │       ├─> Visit variant and create platform-specific module_run:
    │       │   │
    │       │   ├─> AIE2P: new module_run_aie2p(elf, hwctx, ctrl_code_id, config)
    │       │   │   │
    │       │   │   ├─> Store m_config = config (references to ELF data)
    │       │   │   │
    │       │   │   ├─> Create and fill instruction BO:
    │       │   │   │   └─> m_instr_bo = xrt::bo(hwctx, config.instr_data.size(), ...)
    │       │   │   │   └─> Copy: config.instr_data → m_instr_bo
    │       │   │   │   └─> Sync to device: m_instr_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE)
    │       │   │   │
    │       │   │   ├─> Create and fill control packet BO:
    │       │   │   │   └─> m_ctrlpkt_bo = ctrlpkt_bo (pre-created)
    │       │   │   │   └─> Copy: config.ctrl_packet_data → m_ctrlpkt_bo
    │       │   │   │   └─> Sync to device
    │       │   │   │
    │       │   │   ├─> Create preemption BOs (if has_preemption):
    │       │   │   │   └─> m_preempt_save_bo, m_preempt_restore_bo
    │       │   │   │   └─> Fill from config and sync
    │       │   │   │
    │       │   │   ├─> Create scratch pad BOs:
    │       │   │   │   └─> m_scratch_pad_mem_bo (size from config)
    │       │   │   │   └─> m_ctrl_scratch_pad_mem_bo (if size > 0)
    │       │   │   │
    │       │   │   ├─> PDI and ctrlpkt_pm BOs created later (lazy, on-demand)
    │       │   │   │
    │       │   │   └─> Get patcher configs pointer:
    │       │   │       └─> m_patcher_configs = elf_impl->get_patcher_configs(ctrl_code_id)
    │       │   │           └─> Returns &m_arg2patcher[ctrl_code_id]
    │       │   │
    │       │   └─> AIE2PS: new module_run_aie2ps(elf, hwctx, ctrl_code_id, config)
    │       │       │
    │       │       ├─> Store m_config = config
    │       │       │
    │       │       ├─> Create and fill control code BO:
    │       │       │   └─> m_ctrlcode_bo = xrt::bo(hwctx, config.ctrlcodes[ctrl_code_id].size(), ...)
    │       │       │   └─> Copy: config.ctrlcodes[ctrl_code_id] → m_ctrlcode_bo
    │       │       │   └─> Sync to device
    │       │       │
    │       │       ├─> Create and fill dump BO:
    │       │       │   └─> m_dump_bo = xrt::bo(hwctx, config.dump_buf.size(), ...)
    │       │       │   └─> Copy: config.dump_buf → m_dump_bo
    │       │       │   └─> Sync to device
    │       │       │
    │       │       ├─> Control packet BOs created later (lazy, on-demand)
    │       │       │
    │       │       └─> Get patcher configs pointer:
    │       │           └─> m_patcher_configs = elf_impl->get_patcher_configs(ctrl_code_id)
    │       │
    │       └─> Return xrt::module (holding module_run_aie2p or module_run_aie2ps)
    │
    └─> Store module in run object
```

**Key Insights**:
1. `module_config` variant uses references to ELF data (no copying)
2. BOs are created and filled with data from ELF via config references
3. Pointer to shared patcher configs is obtained from `elf_impl`
4. Each `module_run` instance is independent (own BOs, own patcher states)

---

### 4. Argument Setting and Patching

```
User calls run.set_arg(index, bo)
    │
    ├─> Get argument name from index:
    │   └─> arg_name = kernel.get_arg_name(index)
    │
    ├─> Call module.patch(arg_name, index, bo):
    │   └─> module_run::patch(arg_name, index, bo.address())
    │       │
    │       ├─> Get or create symbol_patcher:
    │       │   │
    │       │   ├─> auto it = m_patchers.find(arg_name)
    │       │   │
    │       │   ├─> If patcher doesn't exist:
    │       │   │   │
    │       │   │   ├─> Look up config in shared configs:
    │       │   │   │   └─> auto cfg_it = m_patcher_configs->find(arg_name)
    │       │   │   │
    │       │   │   ├─> Create symbol_patcher:
    │       │   │   │   └─> symbol_patcher patcher(&cfg_it->second)
    │       │   │   │       │
    │       │   │   │       ├─> Store m_config = &cfg_it->second (pointer to shared)
    │       │   │   │       │
    │       │   │   │       └─> Initialize m_states vector:
    │       │   │   │           └─> m_states.resize(m_config->m_patch_configs.size())
    │       │   │   │               └─> One patch_state per patch location
    │       │   │   │
    │       │   │   └─> Store in map:
    │       │   │       └─> m_patchers.emplace(arg_name, std::move(patcher))
    │       │   │
    │       │   └─> Get patcher iterator:
    │       │       └─> it = m_patchers.find(arg_name)
    │       │
    │       ├─> Call patcher.patch_symbol(bo, value, m_first_patch):
    │       │   └─> symbol_patcher::patch_symbol(bo, value, first)
    │       │       │
    │       │       ├─> Get buffer type from config:
    │       │       │   └─> buf_type = m_config->m_buf_type
    │       │       │
    │       │       ├─> For each patch location (i = 0 to m_config->m_patch_configs.size()):
    │       │       │   │
    │       │       │   ├─> Get patch config:
    │       │       │   │   └─> const auto& pc = m_config->m_patch_configs[i]
    │       │       │   │
    │       │       │   ├─> Get target buffer based on buf_type:
    │       │       │   │   ├─> buf_type::ctrltext → target_bo = m_instr_bo
    │       │       │   │   ├─> buf_type::ctrldata → target_bo = m_ctrlpkt_bo
    │       │       │   │   ├─> buf_type::pdi → target_bo = get_pdi_bo(arg_name)
    │       │       │   │   ├─> buf_type::ctrlpkt → target_bo = get_ctrlpkt_bo(arg_name)
    │       │       │   │   └─> etc.
    │       │       │   │
    │       │       │   ├─> Map target buffer to CPU:
    │       │       │   │   └─> uint8_t* base = target_bo.map<uint8_t*>()
    │       │       │   │
    │       │       │   ├─> Calculate patch address in buffer:
    │       │       │   │   └─> uint8_t* patch_addr = base + pc.offset_to_patch_buffer
    │       │       │   │
    │       │       │   ├─> Calculate patch value:
    │       │       │   │   └─> uint64_t patch_value = value + pc.offset_to_base_bo_addr
    │       │       │   │
    │       │       │   ├─> Apply patch based on symbol_type:
    │       │       │   │   ├─> shim_dma_48 → patch_shim48(patch_addr, patch_value)
    │       │       │   │   │   └─> Patch 48-bit address in BD format
    │       │       │   │   │
    │       │       │   │   ├─> control_packet_48 → patch_ctrl48(patch_addr, patch_value)
    │       │       │   │   │   └─> Patch 48-bit address in control packet format
    │       │       │   │   │
    │       │       │   │   ├─> control_packet_57 → patch_ctrl57(patch_addr, patch_value)
    │       │       │   │   │   └─> Patch 57-bit address in control packet format (AIE2PS)
    │       │       │   │   │
    │       │       │   │   ├─> control_packet_57_aie4 → patch_ctrl57_aie4(patch_addr, patch_value)
    │       │       │   │   │   └─> Patch 57-bit address in control packet format (AIE4)
    │       │       │   │   │
    │       │       │   │   ├─> scalar_32bit_kind → patch32(patch_addr, patch_value, pc.mask)
    │       │       │   │   │   └─> Patch masked 32-bit value
    │       │       │   │   │
    │       │       │   │   └─> address_64 → patch64(patch_addr, patch_value)
    │       │       │   │       └─> Patch 64-bit address
    │       │       │   │
    │       │       │   ├─> Update patch state:
    │       │       │   │   └─> m_states[i].dirty = true
    │       │       │   │   └─> Cache original values if needed
    │       │       │   │
    │       │       │   └─> Sync buffer to device:
    │       │       │       └─> If first || need_incremental_sync:
    │       │       │           └─> target_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE, size, offset)
    │       │       │
    │       │       └─> Return
    │       │
    │       ├─> Mark argument as patched:
    │       │   └─> m_patched_args.insert(arg_name)
    │       │
    │       └─> Set dirty flag:
    │           └─> m_dirty = true
    │
    └─> Return
```

**Key Insights**:
1. `symbol_patcher` is created lazily on first use for each argument
2. Each patcher holds a const pointer to shared `patcher_config` from `elf_impl`
3. Each patcher owns its own `patch_state` vector (thread-safe, per-instance)
4. Patching can target different buffer types (instruction, control packet, PDI, etc.)
5. Multiple patch locations per argument are supported

---

### 5. Kernel Execution

```
User calls run.start()
    │
    ├─> Validate all arguments are set:
    │   └─> module_run::sync_if_dirty()
    │       │
    │       ├─> Check all required args patched:
    │       │   └─> For each kernel arg:
    │       │       └─> if arg not in m_patched_args: throw error
    │       │
    │       ├─> If m_dirty:
    │       │   │
    │       │   ├─> Sync all buffers to device:
    │       │   │   ├─> m_instr_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE)
    │       │   │   ├─> m_ctrlpkt_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE)
    │       │   │   ├─> For each PDI BO: sync to device
    │       │   │   └─> etc.
    │       │   │
    │       │   └─> Set m_dirty = false
    │       │
    │       └─> Set m_first_patch = false
    │
    ├─> Build ERT command:
    │   │
    │   ├─> Allocate command buffer
    │   │
    │   ├─> Fill command header (opcode, payload size, etc.)
    │   │
    │   ├─> Fill DPU data:
    │   │   └─> module_run::fill_ert_dpu_data(payload)
    │   │       │
    │   │       ├─> AIE2P:
    │   │       │   └─> Write instruction BO address
    │   │       │   └─> Write control packet BO address (if present)
    │   │       │   └─> Write preemption BO addresses (if present)
    │   │       │   └─> Write scratch pad BO addresses
    │   │       │
    │   │       └─> AIE2PS:
    │   │           └─> Write control code BO address
    │   │           └─> Write control packet BO addresses (if present)
    │   │
    │   └─> Fill CU arguments (if any)
    │
    ├─> Submit command to device:
    │   └─> device->exec_buf(command)
    │
    └─> Return (async execution)

User calls run.wait()
    │
    └─> Wait for command completion from device
```

---

## Key Architectural Benefits

### 1. Clean Separation of Concerns
- **ELF Parsing**: `elf_impl` and derived classes
- **Runtime Execution**: `module_run` and derived classes
- **Patching Logic**: `elf_patcher.*` files
- **User API**: `xrt::elf`, `xrt::module` wrappers

### 2. Platform Abstraction
- Base classes define interface (`elf_impl`, `module_run`)
- Derived classes implement platform-specific behavior
- Factory pattern via `std::variant` for type-safe creation
- Single code path for users regardless of platform

### 3. Thread-Safe Design
- Static config (`patcher_config`) is shared and read-only
- Runtime state (`patch_state`) is per-instance and mutable
- Each `module_run` instance is independent
- Multiple threads can use same ELF safely

### 4. Performance Optimization
- Lazy patcher creation (avoid overhead for unused arguments)
- Lazy BO creation for on-demand buffers (PDI, control packets)
- Dirty tracking reduces unnecessary syncs
- Incremental sync on first patch, full sync before execution

### 5. Maintainability
- Clear class responsibilities
- Easy to add new platforms (add new derived classes)
- Easy to add new symbol types (add new patch function)
- Reduced code duplication (60% reduction in xrt_module.cpp)

---

## Class Dependency Summary

### Compilation Dependencies

```
xrt_elf.cpp depends on:
├── elf_int.h
├── elf_patcher.h
├── ELFIO library
└── xrt/experimental/xrt_elf.h (public API)

xrt_module.cpp depends on:
├── module_int.h
├── elf_int.h
├── elf_patcher.h
├── hw_context_int.h
├── bo_int.h
└── xrt/experimental/xrt_module.h (public API)

elf_patcher.cpp depends on:
├── elf_patcher.h
└── xrt/xrt_bo.h

xrt_hw_context.cpp depends on:
├── hw_context_int.h
└── xrt/xrt_hw_context.h (public API)

xrt_kernel.cpp depends on:
├── module_int.h
├── hw_context_int.h
└── xrt/xrt_kernel.h (public API)
```

### Runtime Dependencies

```
xrt::elf
└── holds shared_ptr<elf_impl>
    ├── elf_aie2p (AIE2P platform)
    │   └── contains map<ctrl_code_id, patcher_config>
    │
    └── elf_aie2ps (AIE2PS platform)
        └── contains map<ctrl_code_id, patcher_config>

xrt::hw_context
└── stores map<kernel_name, xrt::elf>

xrt::module
└── holds shared_ptr<module_impl>
    └── module_run (when associated with hw_context)
        ├── holds shared_ptr<elf_impl>
        ├── holds pointer to patcher_config (from elf_impl)
        ├── owns map<arg_name, symbol_patcher>
        │   └── symbol_patcher
        │       ├── holds pointer to patcher_config (from elf_impl)
        │       └── owns vector<patch_state>
        │
        ├── module_run_aie2p (AIE2P execution)
        │   └── owns multiple xrt::bo objects
        │
        └── module_run_aie2ps (AIE2PS execution)
            └── owns multiple xrt::bo objects
```

---

## File Organization

### Public API Headers
- `xrt/experimental/xrt_elf.h` - ELF user API
- `xrt/experimental/xrt_module.h` - Module user API

### Internal Interface Headers
- `elf_int.h` - ELF internal interface (exposes elf_impl)
- `module_int.h` - Module internal interface (factory functions)
- `hw_context_int.h` - HW context internal interface
- `elf_patcher.h` - Patching system interface

### Implementation Files
- `xrt_elf.cpp` - ELF implementation (elf_impl, elf_aie2p, elf_aie2ps)
- `xrt_module.cpp` - Module implementation (module_impl, module_run, platform variants)
- `elf_patcher.cpp` - Patching implementation (symbol_patcher, patch functions)
- `xrt_hw_context.cpp` - HW context implementation (ELF registration)
- `xrt_kernel.cpp` - Kernel implementation (uses module for execution)

---

## Summary

The XRT ELF and Module architecture provides a clean, maintainable, and performant solution for:
1. **Parsing** platform-specific ELF files
2. **Creating** execution-ready modules with BOs
3. **Patching** kernel arguments into control code
4. **Executing** kernels on AIE devices

The key innovation is the **separation of static configuration from runtime state**, enabling thread-safe sharing of ELF data across multiple module instances while maintaining independent runtime state per instance.

The **platform abstraction** through inheritance and variants allows easy addition of new platforms and provides a unified user API regardless of the underlying hardware.

The **patching system** design separates parsing-time configuration (`patcher_config`) from runtime operations (`symbol_patcher`), enabling efficient, thread-safe patching with minimal overhead.

---

**Document Version**: 1.0  
**Date**: January 28, 2026  
**Commits**: 4e6abd6988fbc33b1c04446fe301aa75d8fab40 to cf66705e5996fd964e27ffbe6755fbf21b4b72d0
