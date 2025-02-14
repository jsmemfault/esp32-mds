# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/tyler/esp/esp-idf/components/bootloader/subproject"
  "/Users/tyler/dev/junk/esp32-mds/build/bootloader"
  "/Users/tyler/dev/junk/esp32-mds/build/bootloader-prefix"
  "/Users/tyler/dev/junk/esp32-mds/build/bootloader-prefix/tmp"
  "/Users/tyler/dev/junk/esp32-mds/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/tyler/dev/junk/esp32-mds/build/bootloader-prefix/src"
  "/Users/tyler/dev/junk/esp32-mds/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/tyler/dev/junk/esp32-mds/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/tyler/dev/junk/esp32-mds/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
