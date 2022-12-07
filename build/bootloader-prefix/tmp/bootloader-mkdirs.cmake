# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/bhuuan/esp/esp-idf/components/bootloader/subproject"
  "D:/rogo/smfan/raw/2022.12.07/ble_smartfan/build/bootloader"
  "D:/rogo/smfan/raw/2022.12.07/ble_smartfan/build/bootloader-prefix"
  "D:/rogo/smfan/raw/2022.12.07/ble_smartfan/build/bootloader-prefix/tmp"
  "D:/rogo/smfan/raw/2022.12.07/ble_smartfan/build/bootloader-prefix/src/bootloader-stamp"
  "D:/rogo/smfan/raw/2022.12.07/ble_smartfan/build/bootloader-prefix/src"
  "D:/rogo/smfan/raw/2022.12.07/ble_smartfan/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/rogo/smfan/raw/2022.12.07/ble_smartfan/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
