#include "startup.hpp"
#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace xsim {
void ensure_socket_parent(const std::string &path) {
  if (path.empty() || path.front() != '/' || path.find('\0') != std::string::npos)
    throw std::invalid_argument("Unix socket requires an absolute filesystem path");
  const auto slash = path.rfind('/');
  const auto filename = path.substr(slash + 1);
  if (filename.empty() || filename == "." || filename == "..")
    throw std::invalid_argument("Unix socket filename is invalid");
  struct Directory {
    int fd;
    ~Directory() { if (fd >= 0) close(fd); }
  } directory{open("/", O_RDONLY | O_DIRECTORY | O_CLOEXEC)};
  if (directory.fd < 0) throw std::runtime_error("cannot open socket parent root");
  std::size_t start = 1;
  while (start < slash) {
    const auto end = path.find('/', start);
    const auto component = path.substr(start, end - start);
    if (component.empty() || component == "." || component == "..")
      throw std::invalid_argument("Unix socket parent components must be explicit directories");
    int next = openat(directory.fd, component.c_str(),
                      O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (next < 0 && errno == ENOENT) {
      if (mkdirat(directory.fd, component.c_str(), 0700) < 0 && errno != EEXIST)
        throw std::runtime_error("cannot create private socket parent");
      next = openat(directory.fd, component.c_str(),
                    O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    }
    if (next < 0)
      throw std::runtime_error("socket parent must be a directory without symbolic links");
    close(directory.fd);
    directory.fd = next;
    start = end + 1;
  }
  struct stat metadata{};
  if (fstat(directory.fd, &metadata) != 0 || metadata.st_uid != geteuid() ||
      (metadata.st_mode & 0777) != 0700)
    throw std::runtime_error("existing socket parent must be owned by this user with mode 0700");
}
} // namespace xsim
