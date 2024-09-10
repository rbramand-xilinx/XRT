/*
 * Copyright 2022 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <thread>

#include "string.h"

// XRT includes
#include "xrt/xrt_device.h"
#include "xrt/xrt_kernel.h"
#include "xrt/xrt_bo.h"
#include "experimental/xrt_elf.h"
#include "experimental/xrt_module.h"
#include "experimental/xrt_ext.h"

#define SELF_TEST          0
#define HOST_APP           1
#define TRANSACTION_LEGACY 2
#define TRANSACTION        3
#define DPU_TRANSACTION_FW 4

void print_dolphin() {
std::cout << "                                       .--.                           " << std::endl;
std::cout << "                _______             .-\"  .\'                         " << std::endl;
std::cout << "        .---u\"\"\"       \"\"\"\"---._  .\"    %                     " << std::endl;
std::cout << "      .\'                        \"--.    %                           " << std::endl;
std::cout << " __.--\'  o                          \"\".. \"                        " << std::endl;
std::cout << "(____.                                  \":                           " << std::endl;
std::cout << " `----.__                                 \".                         " << std::endl;
std::cout << "         `----------__                     \".                        " << std::endl;
std::cout << "               \".   . \"\"--.                 \".                    " << std::endl;
std::cout << "                 \". \". bIt \"\"-.              \".                  " << std::endl;
std::cout << "                   \"-.)        \"\"-.           \".                  " << std::endl;
std::cout << "                                   \"\".         \".                  " << std::endl;
std::cout << "                                      \"\".       \".                 " << std::endl;
std::cout << "                                         \"\".      \".               " << std::endl;
std::cout << "                                            \"\".    \".              " << std::endl;
std::cout << "                      ^~^~^~^~^~^~^~^~^~^~^~^~^\"\".  \"^~^~^~^~^     " << std::endl;
std::cout << "                                            ^~^~^~^  ~^~              " << std::endl;
std::cout << "                                                 ^~^~^~               " << std::endl;
}

size_t get_bin_size(std::string &fname) {
  std::ifstream myfile (fname, std::ifstream::ate | std::ifstream::binary);
  return myfile.tellg();
}

void init_buf_bin(char* buff, size_t bytesize, std::string &filename) {

  std::ifstream ifs(filename, std::ios::in | std::ios::binary);

  if (!ifs.is_open()) {
    std::cout << "Failure opening file " + filename + " for reading!!" << std::endl;
    abort();
  }
  ifs.read((char *)buff, bytesize);
}

void init_buf_bin(xrt::bo &bo, size_t bytesize, std::string &filename) {
  init_buf_bin(bo.map<char*>(), bytesize, filename);
}

int run_test(xrt::device& device, xrt::hw_context& context, const std::string& kernelName, int tid, int it, xrt::module &mod) {
  std::cout << "Thread "<< tid << ", Iteration " << it << " Start" << std::endl;
  std::cout << "Host test code is creating kernel object... " << std::endl;
  auto kernel = xrt::ext::kernel(context, mod, kernelName);

  // ifm BO
  std::string ifm_path = "/proj/xsjhdstaff5/larryliu/testcase/elf_txn_no_cp_stx/ifm.bin";
  size_t ifm_size = get_bin_size(ifm_path);
  if (ifm_size == 0)
    throw std::runtime_error("Error: Why do ifm have zero length?");

  xrt::bo bo_ifm = xrt::ext::bo{device, ifm_size};
  init_buf_bin(bo_ifm, ifm_size, ifm_path);
  bo_ifm.sync(XCL_BO_SYNC_BO_TO_DEVICE);

  // wts BO
  std::string wts_path = "/proj/xsjhdstaff5/larryliu/testcase/elf_txn_no_cp_stx/wts.bin";
  size_t wts_size = get_bin_size(wts_path);
  if (wts_size == 0)
    throw std::runtime_error("Error: Why do wts have zero length?");

  xrt::bo bo_wts = xrt::ext::bo{device, wts_size};
  init_buf_bin(bo_wts, wts_size, wts_path);
  bo_wts.sync(XCL_BO_SYNC_BO_TO_DEVICE);

  // ofm BO (same as golden)
  std::string ofm_path = "/proj/xsjhdstaff5/larryliu/testcase/elf_txn_no_cp_stx/ofm.bin";
  size_t ofm_size = get_bin_size(ofm_path);
  if (ofm_size == 0)
    throw std::runtime_error("Error: Why do ofm have zero length?");

  xrt::bo bo_ofm = xrt::ext::bo{device, ofm_size};

  // Set kernel argument and trigger it to run
  uint64_t opcode = TRANSACTION;
  auto run = kernel(opcode, 0, 0, bo_ifm, bo_wts, bo_ofm, 0, 0);

  // Wait for kernel to be done
  run.wait2();

  // Sync output frame back to host
  bo_ofm.sync(XCL_BO_SYNC_BO_FROM_DEVICE);

  // compare golden with output
  auto out = (uint8_t *)bo_ofm.map<uint8_t *>();
  auto golden = new char[ofm_size];
  init_buf_bin(golden, ofm_size, ofm_path);
  int count = 0;
  for (size_t i = 0; i < ofm_size; i++) {
	  auto ofm_byte = (uint8_t)out[i];
	  auto golden_byte = (uint8_t)golden[i];
	  if (golden_byte != ofm_byte)
		  count++;
  }

  if (count == 0)
    print_dolphin();
  else
    std::cout << "Test FAILS. " << count << " mismatches." << std::endl;

  return 0;
}

int run_test_iterations(const std::string &xclbinFileName, xrt::device &device, int tid, int it_max) {

  std::cout << "===== Thread " << tid << "Start =====" << std::endl;

  //Load Xclbin Object
  std::cout << "Host test code is loading xclbin object..." << xclbinFileName << std::endl;
  auto xclbin = xrt::xclbin(xclbinFileName);

  // Determine The DPU Kernel Name
  auto xkernels = xclbin.get_kernels();
  auto xkernel = *std::find_if(xkernels.begin(), xkernels.end(), [](xrt::xclbin::kernel& k) {
    auto name = k.get_name();
    return name.rfind("DPU",0) == 0; // Starts with "DPU"
  });
  auto kernelName = xkernel.get_name();

  std::cout << "Host test code found kernel: " << kernelName << std::endl;

  std::cout << "Host code is registering xclbin to the device..." << std::endl;
  device.register_xclbin(xclbin);

  std::cout << "Host code create elf..." << std::endl;
  xrt::elf elf{"/proj/xsjhdstaff5/larryliu/testcase/elf_txn_no_cp_stx/extra_info/no-ctrl-packet.elf"};

  std::cout << "Host code create module..." << std::endl;
  xrt::module mod{elf};

  std::cout << "Host code is creating hw_context..." << std::endl;
  xrt::hw_context context(device, xclbin.get_uuid());

  // Start Iteration
  // All iterations in the same thread share the same hw context
  for (int i = 0; i < it_max; i++) {
    run_test(device, context, kernelName, tid, i, mod);
    std::cout << "Thread " << tid << ", Iteration "<< i << " Finished" << std::endl;
  }
  std::cout << "===== Thread "<< tid << " Finished  =====" <<std::endl;

  return 0;
}

int main(int argc, char **argv) {

  std::cout << "Host test code start..." << std::endl;

  std::string xclbinFileName = "/proj/xsjhdstaff5/larryliu/testcase/elf_txn_no_cp_stx/design.xclbin";

  // Set the number of threads and iterations
  // Default: 1 thread, 1 iteration
  int num_thread, it_max;

  num_thread = 1;
  it_max = 1;

  std::cout << "Host test code is creating device object..." << std::endl;
  unsigned int device_index = 0;
  auto device = xrt::device(device_index);

  std::vector<std::thread> threads;
  for (int i=0; i < num_thread; i++)
    threads.emplace_back(std::thread(run_test_iterations, xclbinFileName, std::ref(device), i, it_max));

  for (auto& th : threads)
    th.join();

  return 0;
}
