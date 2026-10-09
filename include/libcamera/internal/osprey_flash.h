/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once
#include <cerrno>
#include <fcntl.h>
#include <string>
#include <unistd.h>

inline bool ospreyFlashAvailable(const std::string &id)
{
    return id.find("rear-camera@2d") != std::string::npos &&
        access("/sys/class/leds/osprey:flash/brightness", W_OK) == 0;
}

inline int ospreyFlashWrite(const char *attribute, bool enabled)
{
    const std::string path = std::string("/sys/class/leds/osprey:flash/") + attribute;
    int fd = open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0)
        return -errno;
    char value = enabled ? (std::string(attribute) == "brightness" ? '3' : '1') : '0';
    int ret = write(fd, &value, 1) == 1 ? 0 : -errno;
    close(fd);
    return ret;
}
