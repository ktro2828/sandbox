#include <algorithm>
#include <fstream>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <iterator>
#include <cassert>
#include <memory>

#include "ioHelper.hpp"

namespace nvinfer1
{
  std::string getBasename(std::string const &path)
  {
    #ifdef _WIN32
    constexpr char SEPARATOR = '\\';
    #else
    constexpr char SEPARATOR = '/';
    #endif
    int baseId = path.rfind(SEPARATOR) + 1;
    return path.substr(baseId, path.rfind('.') - baseId);
  }

  std::ostream& operator<<(std::ostream &o, const nvinfer1::ILogger::Severity severity)
  {
    switch (severity)
      {
      case nvinfer1::ILogger::Severity::kINTERNAL_ERROR:
	o << "INTERNAL_ERROR";
	break;
      case nvinfer1::ILogger::Severity::kERROR:
	o << "ERROR";
	break;
      case nvinfer1::ILogger::Severity::kWARNING:
	o << "WARNING";
	break;
      case nvinfer1::ILogger::Severity::kINFO:
	o << "INFO";
	break;
      }
    return o;
  }
} // namespace nvinfer1
