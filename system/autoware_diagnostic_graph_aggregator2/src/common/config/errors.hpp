// Copyright 2023 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMMON__CONFIG__ERRORS_HPP_
#define COMMON__CONFIG__ERRORS_HPP_

#include <stdexcept>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

struct Exception : public std::runtime_error
{
  using runtime_error::runtime_error;
};

struct UnknownSubstitution : public Exception
{
  using Exception::Exception;
};

struct SameFileFound : public Exception
{
  using Exception::Exception;
};

struct FileNotFound : public Exception
{
  using Exception::Exception;
};

struct PathConflict : public Exception
{
  using Exception::Exception;
};

struct LinkNotFound : public Exception
{
  using Exception::Exception;
};

struct LinkLoopFound : public Exception
{
  using Exception::Exception;
};

struct UnitLoopFound : public Exception
{
  using Exception::Exception;
};

/*
struct FileNotFound : public Exception
{
  FileNotFound(const TreePath & path, const std::string & file)
  : Exception(format(path, file))
  {
  }
  static std::string format(const TreePath & path, const std::string & file)
  {
    return "file is not found: " + file + path.text();
  }
};

struct FieldNotFound : public Exception
{
  explicit FieldNotFound(const TreePath & path) : Exception(format(path)) {}
  static std::string format(const TreePath & path)
  {
    return "required field is not found: " + path.name() + path.text();
  }
};

struct InvalidType : public Exception
{
  explicit InvalidType(const TreePath & path) : Exception(format(path)) {}
  static std::string format(const TreePath & path)
  {
    return "field is not a list type: " + path.name() + path.text();
  }
};

struct ModeNotFound : public Exception
{
  explicit ModeNotFound(const std::string & path) : Exception(format(path)) {}
  static std::string format(const std::string & path) { return "mode path is not found: " + path; }
};

struct UnknownUnitType : public Exception
{
  explicit UnknownUnitType(const TreePath & path, const std::string & type)
  : Exception(format(path, type))
  {
  }
  static std::string format(const TreePath & path, const std::string & type)
  {
    return "unknown unit type: " + type + path.text();
  }
};

struct GraphStructure : public Exception
{
  using Exception::Exception;
};
*/

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__ERRORS_HPP_
