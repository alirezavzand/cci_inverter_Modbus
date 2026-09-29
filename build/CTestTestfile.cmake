# CMake generated Testfile for 
# Source directory: C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2
# Build directory: C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test(huawei_driver "C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/build/Debug/test_huawei_driver.exe")
  set_tests_properties(huawei_driver PROPERTIES  _BACKTRACE_TRIPLES "C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/CMakeLists.txt;103;add_test;C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test(huawei_driver "C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/build/Release/test_huawei_driver.exe")
  set_tests_properties(huawei_driver PROPERTIES  _BACKTRACE_TRIPLES "C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/CMakeLists.txt;103;add_test;C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test(huawei_driver "C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/build/MinSizeRel/test_huawei_driver.exe")
  set_tests_properties(huawei_driver PROPERTIES  _BACKTRACE_TRIPLES "C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/CMakeLists.txt;103;add_test;C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test(huawei_driver "C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/build/RelWithDebInfo/test_huawei_driver.exe")
  set_tests_properties(huawei_driver PROPERTIES  _BACKTRACE_TRIPLES "C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/CMakeLists.txt;103;add_test;C:/MY/GIT/cci-software/cci_inverter_gateway_v1_2/CMakeLists.txt;0;")
else()
  add_test(huawei_driver NOT_AVAILABLE)
endif()
