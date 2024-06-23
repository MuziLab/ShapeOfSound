# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/ESP/components/bootloader/subproject"
  "D:/vs_code/esp_and_espidf/course-project/build/bootloader"
  "D:/vs_code/esp_and_espidf/course-project/build/bootloader-prefix"
  "D:/vs_code/esp_and_espidf/course-project/build/bootloader-prefix/tmp"
  "D:/vs_code/esp_and_espidf/course-project/build/bootloader-prefix/src/bootloader-stamp"
  "D:/vs_code/esp_and_espidf/course-project/build/bootloader-prefix/src"
  "D:/vs_code/esp_and_espidf/course-project/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/vs_code/esp_and_espidf/course-project/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/vs_code/esp_and_espidf/course-project/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
