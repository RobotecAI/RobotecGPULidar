#pragma once
#include <string>
#include <mutex>

// Thread-safe (even if the rest of the code is not)
std::string generate_simple_uid()
{
  static std::mutex mutex;
  static long int uid_int = 0;

  std::unique_lock<std::mutex> lock(mutex);
  std::string uid = std::to_string(uid_int);
  uid_int++;
  return uid;
}
