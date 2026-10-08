#pragma once
#include <string>

namespace xsim {
// Create missing private directories only. Existing directories and other
// filesystem objects are never chmod'ed, replaced or removed.
void ensure_socket_parent(const std::string &path);
} // namespace xsim
