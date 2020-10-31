# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.16.4)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(ode-populate NONE)

include(ExternalProject)
ExternalProject_Add(ode-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://bitbucket.org/odedevs/ode/downloads/ode-0.16.1.tar.gz" "URL_HASH" "MD5=0af3efe98f19163c04083c554afff629" "UPDATE_COMMAND"
                    SOURCE_DIR          "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/ode-src"
                    BINARY_DIR          "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/ode-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)
