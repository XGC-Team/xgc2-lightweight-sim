#include "io/startup.hpp"
#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace {
mode_t mode(const std::filesystem::path &path) {
  struct stat value{};
  assert(lstat(path.c_str(), &value) == 0);
  return value.st_mode & 0777;
}
template <class Function> void rejects(Function function) {
  bool rejected = false;
  try { function(); } catch (const std::exception &) { rejected = true; }
  assert(rejected);
}
}

int main() {
  char pattern[] = "/tmp/xsim-startup-test-XXXXXX";
  const char *created = mkdtemp(pattern);
  assert(created);
  const std::filesystem::path root(created);
  struct Remove { std::filesystem::path root; ~Remove() { std::filesystem::remove_all(root); } } remove{root};
  const auto socket = root / "new" / "nested" / "world.sock";
  xsim::ensure_socket_parent(socket.string());
  assert(mode(root / "new") == 0700 && mode(root / "new" / "nested") == 0700);
  assert(!std::filesystem::exists(socket));
  xsim::ensure_socket_parent(socket.string());
  assert(mode(root / "new" / "nested") == 0700);
  const auto public_directory = root / "existing";
  assert(mkdir(public_directory.c_str(), 0755) == 0);
  assert(chmod(public_directory.c_str(), 0755) == 0);
  rejects([&] { xsim::ensure_socket_parent((public_directory / "world.sock").string()); });
  assert(mode(public_directory) == 0755);
  const auto linked = root / "linked";
  assert(symlink((root / "new").c_str(), linked.c_str()) == 0);
  rejects([&] { xsim::ensure_socket_parent((linked / "nested" / "world.sock").string()); });
  assert(std::filesystem::is_symlink(linked));
  std::ofstream(root / "file") << "foreign-data";
  rejects([&] { xsim::ensure_socket_parent((root / "file" / "world.sock").string()); });
  assert(std::filesystem::file_size(root / "file") == 12);
  for (const auto &path : {"", "relative.sock", "/", "/tmp/../world.sock", "/tmp/./world.sock", "/tmp//world.sock"})
    rejects([&] { xsim::ensure_socket_parent(path); });
  std::cout << "private parent creation, no chmod, symlink/file refusal and explicit path checks passed\n";
}
