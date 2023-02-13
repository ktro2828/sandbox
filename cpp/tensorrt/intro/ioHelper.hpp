
#ifndef __IO_HELPER_HPP__
#define __IO_HELPER_HPP__

#include <NvInfer.h>
#include <iostream>
#include <vector>
#include <string>

namespace nvinfer1
{

  std::ostream& operator<<(std::ostream &o, const nvinfer1::ILogger::Severity severity);

  class Logger : public nvinfer1::ILogger
  {
  private:
    bool verbose_{false};

  public:
    explicit Logger(bool verbose) : verbose_(verbose) {}

    virtual void log(Severity severity, const char* msg) noexcept override
    {
      std::cerr << severity << ": " << msg << std::endl;
    }
  }; // class Logger

  template <typename T>
  struct Deleter
  {
    void operator()(T* t) const
    {
      if (obj) {
	delete obj;
      };
    }
  }; // struct Destroy

  std::string getBasename(std::string const &path);

} // namespace nvinfer1

#endif // __IO_HELPER_HPP__
