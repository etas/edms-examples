## Develop

In this section, we will explore how an application is typically developed using the YAAA (Yaml As an Architecture) language and C++.

### YAAA modeling elements and project structure

YAAA provides a set of YAML-based templates for modeling the software architecture of ADAS / AD systems.

The modeling elements are:

- **Interfaces** define data used for communication (defined in sub-directory `interfaces`)
- **Runnables** contain business logic in the form of C++ code (defined in sub-directory `runnables`)
- **Activities** define and represent executables that are assigned to POSIX processes (defined in sub-directory `activities`)
- **Activity graphs** define complete applications through instantiation of activities (defined in sub-directory `deployment` as YAML file with `_graph`)
- **Deployment** maps the software applications to concrete hardware entities (defined in sub-directory `deployment`)

The modeling elements are each specified in separate YAML files. These are typically organized in a corresponding directory structure within a project. 

```
basic_development_workflow/
├── CMakeLists.txt
├── conanfile.py
├── interfaces/
│   ├── CMakeLists.txt
│   ├── ExampleInterfaces.interfaces.yaml        # interface definitions
│   └── interfaces_include/
│       └── example_interfaces/
│           └── message.hpp                      # C++ struct for Message
├── runnables/
│   ├── CMakeLists.txt
│   ├── Producer.runnable.yaml                   # ports and internal state
│   ├── Consumer.runnable.yaml                   # ports and policy
│   ├── runnables_include/
│   │   └── example_runnables/
│   │       └── internal_state.hpp               # InternalState struct
│   └── src/
│       ├── producer.cpp                         # onInit / onUpdate business logic
│       └── consumer.cpp
├── activities/
│   ├── CMakeLists.txt
│   ├── Producer.activity.yaml                   # stimulus, connections, runnable instances
│   └── Consumer.activity.yaml
└── deployment/
    ├── CMakeLists.txt
    ├── HelloWorld.activity_graph.yaml            # instantiates and connects activities
    └── Deployment.deploy.yaml                   # maps to hardware + measurement config
```

Go through the directory structure and have a look at the different files in the above mentioned directories. 

### Business logic implementation in C++

The business logic is implemented in the in C++ in the Runnables which can be found '\runnables\src\'

C++ code for the Producer

```cpp
#include "Producer.hpp"

void Producer_onInit(const Producer_InStruct&, mwala::generic_io::State<Producer_State>&, Producer_RunContext&)
{
}

void Producer_onUpdate(const Producer_InStruct&,
                       Producer_OutStruct& out,
                       mwala::generic_io::State<Producer_State>& state,
                       Producer_RunContext&)
{
    ++state.get().counter;

    out.m_output.get().number = state.get().counter;
    out.m_output.setSendState(mwala::generic_io::SendState::SEND);

    LOG_INFO("sending number: {}", out.m_output.get().number);
}
```

C++ code for the Consumer

```cpp
#include "Consumer.hpp"

void Consumer_onInit(const Consumer_InStruct&, mwala::generic_io::State<Consumer_State>&, Consumer_RunContext&)
{
}

void Consumer_onUpdate(const Consumer_InStruct& in,
                       Consumer_OutStruct&,
                       mwala::generic_io::State<Consumer_State>&,
                       Consumer_RunContext&)
{
    if (in.m_input.empty())
    {
        LOG_INFO("m_input is empty");
    }
    else
    {
        for (auto& x : in.m_input)
        {
            LOG_INFO("received number: {}", x.get().number);
            if (x.get().number % 2 == 0)
            {
                LOG_INFO("number is even");
            }
            else
            {
                LOG_INFO("number is odd");
            }
        }
    }
}
```

Each Runnable has two functions which are called by the middleware.

- The `onInit()` function is called once upon initialization.
- The `onUpdate()` function is called whenever the runnable gets executed. It takes the values from the InStruct and the internal state and produces the values in the OutStruct.

Note that the `onInit()` function is not implemented as no initialization is necessary in the simple example.

### **Build configuration with Conan and CMake**

The build configuration for the project is done using Conan and CMake.

In the `CMakeLists.txt` files of the sub-directories of the project, the YAAA modeling files and C++ files are added to the build system. Note that the SDK provides a set of macros - provided through YAAA CMake - to handle the contents more easily.

```cmake
cmake_minimum_required(VERSION 3.16)
project(hello_world)

find_package(aos_build_tools REQUIRED)
find_package(aos_runtime_prod REQUIRED)
find_package(aos_runtime_dev REQUIRED)

include(yaaa_init)
yaaa_init(
  ROOT_FILE "deployment/Deployment.deploy.yaml"
  YAAA_IMPORT_DIRS
    deployment
    activities
    runnables
    interfaces
)

add_subdirectory(interfaces)
add_subdirectory(runnables)
add_subdirectory(activities)
add_subdirectory(deployment)
```

In the `conanfile.py`, project-wide settings for the actual build are defined.

```python
options = {
    "CODE_GENERATOR": ["yaaac2"],
    "MIDDLEWARE_TARGET": ["carma_0_22", "recall", "noviroc"],
    "LOGGING_FRAMEWORK": ["no_console", "console"],
}
default_options = {
    "CODE_GENERATOR": "yaaac2",
    "MIDDLEWARE_TARGET": "noviroc",
    "LOGGING_FRAMEWORK": "console",
}
```

### Tips

- When changing files, regularly save your work and commit changes to version control.
- Use the YAAAVis extension for better development experience.

Let's proceed to the next section where we will build our project.

Previous section: [Introduction](../readme.md) | Next section: [Build](03-build.md)
