#include "core_stub.hpp"

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

namespace mobile_ros {
namespace {
constexpr auto kProjectName = "MobileROS";
}

std::string version() {
  // A lightweight semantic version string that does not depend on git metadata.
  return "0.1.0-core";
}

void print_example_banner() {
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::cout << "[" << kProjectName << "] core-only build (" << version()
            << ") initialized at " << std::ctime(&now) << std::endl;
  std::cout << "This binary is built with ENABLE_OAI=OFF. Radio hardware is mocked." << std::endl;
}

}  // namespace mobile_ros
