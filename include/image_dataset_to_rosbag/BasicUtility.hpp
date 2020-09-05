/**
 * @file    BasicUtility.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace utils
{
inline std::vector<std::string> split(const std::string& s, const char delimiter)
{
    std::stringstream ss(s);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, delimiter)) {
        tokens.emplace_back(token);
    }
    return tokens;
}

inline bool fileExists(const std::string& path)
{
    return fs::exists(path);
}

inline bool directoryExists(const std::string& path)
{
    return fs::is_directory(path);
}

inline std::vector<std::string> parseMetaDataFile(const std::string& metaDataFilePath)
{
    std::ifstream inFile;
    inFile.open(metaDataFilePath);
    if (!inFile) {
        throw std::runtime_error("Unable to open " + metaDataFilePath + "\n");
    }

    std::stringstream buffer;
    buffer << inFile.rdbuf();

    return split(buffer.str(), '\n');
}
}  // namespace utils
