/**
 * @file    ParamConfig.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <exception>
#include <string>

namespace utils
{
struct ParamConfigError : public std::exception {
 public:
    explicit ParamConfigError(const std::string& errorMessage)
        : m_errorMessage(errorMessage)
    {
    }

    const char* what() const noexcept override
    {
        return m_errorMessage.c_str();
    }

 private:
    const std::string m_errorMessage;
};

template <typename ParamT> void validateParam(const ParamT& params);
}  // namespace utils
