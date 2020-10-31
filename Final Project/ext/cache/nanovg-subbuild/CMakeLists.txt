# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.16.4)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(nanovg-populate NONE)

include(ExternalProject)
ExternalProject_Add(nanovg-populate
                     "UPDATE_DISCONNECTED" "False" "GIT_REPOSITORY" "https://github.com/memononen/nanovg.git" "GIT_TAG" "1f9c8864fc556a1be4d4bf1d6bfe20cde25734b4" "UPDATE_COMMAND"
                    SOURCE_DIR          "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/nanovg-src"
                    BINARY_DIR          "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/nanovg-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)
