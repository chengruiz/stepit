# Plugin Mechanism

StepIt employs a modular plugin architecture to extend functionalities like robot backends, control inputs, and policies.

## 1. Plugin Root Index

At configure time, StepIt treats its built-in `plugin/` directory and every CMake `STEPIT_PLUGIN_DIRS` entry as a
plugin root. Each root must contain `plugins.cmake`, which sets `STEPIT_PLUGIN_PATHS` to the plugin directories it
contains. Relative paths are resolved from the root; absolute paths are also allowed.

```cmake
# plugin_root/plugins.cmake
set(STEPIT_PLUGIN_PATHS
    field_base
    my_plugin
)
```

## 2. Plugin Directory

A typical plugin directory layout:

```text
my_plugin/
├── plugin.cmake
├── CMakeLists.txt
├── include/
│   └── stepit/
│       └── my_plugin/
│           └── my_plugin.h
└── src/
    └── my_plugin.cpp
```

## 3. Dependencies & Build (`CMakeLists.txt`)

StepIt discovers only the directories listed in a plugin root's `plugins.cmake`.
Each listed directory must contain `plugin.cmake`, which declares the plugin name and dependencies with
`stepit_declare_plugin(...)`.

Use plain CMake logic in the manifest:

```cmake
stepit_declare_plugin(NAME my_plugin DEPENDS field_base)
find_package(Python3 QUIET COMPONENTS Interpreter Development)
if (NOT Python3_FOUND)
  stepit_plugin_mark_unbuildable("Missing Python3 interpreter or development files.")
  return()
endif ()
```

Then define a normal CMake library target and register it with StepIt.

```cmake
cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_example)

stepit_add_plugin(stepit_plugin_example)

target_sources(stepit_plugin_example
    PRIVATE
      src/my_plugin.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/my_plugin/my_plugin.h
)

target_link_libraries(stepit_plugin_example PUBLIC
    stepit_core
    stepit_plugin_field_base
)

install(TARGETS stepit_plugin_example
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
```

> **Note**: The library target must be named `stepit_plugin_<plugin name>`.

## 4. Implementing Interfaces

Inherit from the base classes defined in `stepit` headers and implement the required virtual methods.

```cpp
// my_plugin.h
#include <stepit/robot.h>

namespace stepit {
namespace my_plugin {
class MyRobot : public RobotApi {
 public:
  explicit MyRobot() : RobotApi("my_robot") {}
  void getControl(bool enable) override {}
  void setSend(const LowCmd &) override {}
  void getRecv(LowState &) override {}
  void send() override {}
  void recv() override {}
};
} // namespace my_plugin
} // namespace stepit
```


## 5. Registration

Register your implementations using the provided macros in your `.cpp` file with `STEPIT_REGISTER_*`, for example:

```cpp
#include <stepit/my_plugin/robot.h>

namespace stepit {
// Register custom robot API with default priority
STEPIT_REGISTER_ROBOTAPI(my_robot, kDefPriority, RobotApi::make<MyRobot>);
}  // namespace stepit
```

## 6. Lifecycle Hooks (Optional)

You can export C-style functions for initialization and cleanup:

- `extern "C" int stepit_plugin_init(int &argc, char *argv[])`: Called on load. Use to parse custom arguments.
- `extern "C" void stepit_plugin_cleanup()`: Called on unload.

## 7. Building & Loading

1.  Build your plugin using CMake.
2.  Ensure the compiled `.so` file is in a path searchable by StepIt. By default, it searches the following directories:
    - `<executable_dir>/`
    - `<executable_dir>/../lib/`
    - `<executable_dir>/../../lib/`
    - Other directories specified by the `STEPIT_EXTRA_PLUGIN_DIRS` environment variable.
3.  The `PluginManager` will automatically discover and load libraries matching `libstepit_plugin_*.so`.
