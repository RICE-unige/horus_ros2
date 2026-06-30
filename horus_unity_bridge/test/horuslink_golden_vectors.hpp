// Copyright 2026 RICE Lab, University of Genoa
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// SPDX-FileCopyrightText: 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace horus_unity_bridge::horuslink
{

inline std::vector<uint8_t> load_horuslink_golden_vector(const std::string & name)
{
  const std::string path =
    std::string(HORUS_UNITY_BRIDGE_TEST_SOURCE_DIR) + "/test/golden_vectors/" + name + ".hex";
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open HorusLink golden vector: " + path);
  }

  std::vector<uint8_t> output;
  std::string token;
  while (file >> token) {
    output.push_back(static_cast<uint8_t>(std::stoul(token, nullptr, 16)));
  }

  return output;
}

}  // namespace horus_unity_bridge::horuslink
