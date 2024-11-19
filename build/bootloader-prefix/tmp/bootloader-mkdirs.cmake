# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Dericc/esp/esp-idf/components/bootloader/subproject"
  "E:/KULIAH-E/CAKSA/Remote-Transmitter/build/bootloader"
  "E:/KULIAH-E/CAKSA/Remote-Transmitter/build/bootloader-prefix"
  "E:/KULIAH-E/CAKSA/Remote-Transmitter/build/bootloader-prefix/tmp"
  "E:/KULIAH-E/CAKSA/Remote-Transmitter/build/bootloader-prefix/src/bootloader-stamp"
  "E:/KULIAH-E/CAKSA/Remote-Transmitter/build/bootloader-prefix/src"
  "E:/KULIAH-E/CAKSA/Remote-Transmitter/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/KULIAH-E/CAKSA/Remote-Transmitter/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/KULIAH-E/CAKSA/Remote-Transmitter/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
