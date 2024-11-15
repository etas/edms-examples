__copyright__ = """
==============================================================================
 @copyright (c) 2024 ETAS GmbH
==============================================================================
"""
import os
from conan.tools.cmake import CMakeDeps
from conans import CMake, ConanFile

#for copying files from the install to the build folder
import shutil
from conans import ConanFile, tools

class Example(ConanFile):
    name = "hello_world"
    version = "0.0.1"
    license = "Proprietary"
    url = "None"
    author = "ETAS GmbH"
    description = ""
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake_find_package", "virtualenv", "virtualrunenv", "json"
    keep_imports = True
    exports_sources = "*"

    # aos version and channel for finding required aos packages
    # set environment variables AOS_VERSION or AOS_CHANNEL for overwriting them
    # e.g. export AOS_CHANNEL=dev
    aos_version = os.getenv("AOS_VERSION","0.33.0")
    aos_channel = os.getenv("AOS_CHANNEL","eap")

    options = {
        "ROS": ["ON", "OFF"],
        "RECALL": ["ON", "OFF"],
        "ROS_INITIALIZATION": ["ON", "OFF"],
        "CODE_GENERATOR": ["yaaac2"],
        "MIDDLEWARE_TARGET": ["carma_0_22"],
        "LOGGING_FRAMEWORK": ["console", "console_and_mta"],
    }
    default_options = {
        "ROS": "OFF",
        "RECALL": "ON",
        "ROS_INITIALIZATION": "OFF",
        "CODE_GENERATOR": "yaaac2",
        "MIDDLEWARE_TARGET": "carma_0_22",
        "LOGGING_FRAMEWORK": "console_and_mta",
    }

    def build_requirements(self):
        self.build_requires(f"aos_build_tools/{self.aos_version}@aos/{self.aos_channel}")

    def generate(self):
        if hasattr(self, "settings_build"):
            cmakedeps = CMakeDeps(self)
            # generate the config files for the tool require
            cmakedeps.build_context_activated.append("aos_build_tools")
            cmakedeps.generate()

    def requirements(self):
        self.requires(f"aos_runtime_dev/{self.aos_version}@aos/{self.aos_channel}")
        self.requires(f"aos_runtime_prod/{self.aos_version}@aos/{self.aos_channel}")
        if self.settings.arch == "x86_64":
            self.requires(f"aos_analysis_bundle/{self.aos_version}@aos/{self.aos_channel}")

    def build(self):
        cmake = CMake(self)

        # iterate over all LOCAL options and add them to all CMake definitions
        for opt_name, opt_value in self.options.items():
            cmake.definitions[opt_name] = opt_value
        cmake.definitions["BUILD_WARNINGS_AS_ERRORS"] = True

        # for the following CMake definitions it makes no sense to define them as
        # options, since e.g. a package built with or without ccache does not differ
        # in terms of output artifacts. They can be enabled here if required
        # Put the following lines back in to activate them and tweak them:
        # cmake.definitions["CCACHE"] = "ON"
        # cmake.definitions["VISU_LANDING_PAGE_RESOURCES"] = ""

        cmake.configure()
        cmake.build()

        if self.install_folder != self.build_folder:
            # copy files from the install to the build folder
            install_lib_path = os.path.join(self.install_folder, "lib")
            install_bin_path = os.path.join(self.install_folder, "bin")
            build_lib_path = os.path.join(self.build_folder, "lib")
            build_bin_path = os.path.join(self.build_folder, "bin")
            if not os.path.exists(build_lib_path):
                os.makedirs(build_lib_path)        
            if not os.path.exists(build_bin_path):
                os.makedirs(build_bin_path)        
            for file_name in os.listdir(install_lib_path):
                full_file_name = os.path.join(install_lib_path, file_name)
                if os.path.isfile(full_file_name):
                    shutil.copy(full_file_name, build_lib_path)
            for file_name in os.listdir(install_bin_path):
                full_file_name = os.path.join(install_bin_path, file_name)
                if os.path.isfile(full_file_name):
                    shutil.copy(full_file_name, build_bin_path)

    def package(self):
        self.copy("*", excludes=("test_package"))

    def imports(self):
        self.copy("*.so", dst="lib", root_package="aos_runtime_dev", keep_path=False)
        self.copy("*recompute_recall_player", dst="bin", root_package="aos_runtime_dev", keep_path=False)
        self.copy("bin/esme", dst="bin", root_package="aos_runtime_prod", keep_path=False)
        self.copy("bin/mta_raw_output_gateway", dst="bin", root_package="aos_runtime_dev", keep_path=False) 
        self.copy("bin/mta_ros1_output_gateway", dst="bin", root_package="aos_runtime_dev", keep_path=False)
