# CMake generated Testfile for 
# Source directory: /home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann
# Build directory: /home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/copyright.xunit.xml" "--package-name" "vesc_ackermann" "--output-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/ament_copyright/copyright.txt" "--command" "/opt/ros/foxy/bin/ament_copyright" "--xunit-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_copyright/cmake/ament_copyright.cmake;41;ament_add_test;/opt/ros/foxy/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;18;ament_copyright;/opt/ros/foxy/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;45;ament_auto_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/cppcheck.xunit.xml" "--package-name" "vesc_ackermann" "--output-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/foxy/bin/ament_cppcheck" "--xunit-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/cppcheck.xunit.xml" "--include_dirs" "/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;61;ament_add_test;/opt/ros/foxy/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;83;ament_cppcheck;/opt/ros/foxy/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;45;ament_auto_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;0;")
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/cpplint.xunit.xml" "--package-name" "vesc_ackermann" "--output-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/ament_cpplint/cpplint.txt" "--command" "/opt/ros/foxy/bin/ament_cpplint" "--xunit-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_cpplint/cmake/ament_cpplint.cmake;68;ament_add_test;/opt/ros/foxy/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;35;ament_cpplint;/opt/ros/foxy/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;45;ament_auto_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/lint_cmake.xunit.xml" "--package-name" "vesc_ackermann" "--output-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/foxy/bin/ament_lint_cmake" "--xunit-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;41;ament_add_test;/opt/ros/foxy/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/foxy/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;45;ament_auto_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/uncrustify.xunit.xml" "--package-name" "vesc_ackermann" "--output-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/foxy/bin/ament_uncrustify" "--xunit-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;65;ament_add_test;/opt/ros/foxy/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;34;ament_uncrustify;/opt/ros/foxy/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;45;ament_auto_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/xmllint.xunit.xml" "--package-name" "vesc_ackermann" "--output-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/ament_xmllint/xmllint.txt" "--command" "/opt/ros/foxy/bin/ament_xmllint" "--xunit-file" "/home/nx-ros2/varun_ws/f1tenth_ws/build/vesc_ackermann/test_results/vesc_ackermann/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/foxy/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/foxy/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;45;ament_auto_package;/home/nx-ros2/varun_ws/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/CMakeLists.txt;0;")
