/*
 * Cloud-based Object Recognition Engine (CORE)
 *
 */

#ifndef UTILS_H
#define UTILS_H

#include <cerrno>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <core/console/print.h>

void replaceExt (std::string &s, const std::string &ext);
int getCategories (const std::string file_name, std::vector<std::string> &categories);
int getData (const std::string dir_name, std::vector<std::string> &data);

#endif  // UTILS_H
