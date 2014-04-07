^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openhrp3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Give installed libraries execute permissions
* Fix python syntax errors
  You cannot define a function called exec. This patch renames it to Exec.
* Handle non-existent lsb-release file
  This file is not present on Fedora systems.
* test_openhpr3.py: add test code to check hrpsys-base
* add test code to check if file exists
* add test status from jenkins.ros.org
* add start_omninames.sh start starts omniNames for test code, use port 2809 for test
* add test sample1.wrl location
* (Makefile.openhrp3) touch patched_no_makefile to avoid compile twice
* add PKG_CONFIG_PATH for rosbuild environment
* (.travis.yml) add rosbuild/deb test
* (CMakeLists.txt) add rostest
* (`#32 <https://github.com/start-jsk/openhrp3/issues/32>`_) add roslang for manifest.xml and package.xml
* (`#24 <https://github.com/start-jsk/openhrp3/issues/24>`_) add rosbuild, see https://github.com/ros/ros/issues/47
* check rosdep until it succeeded
* Fix cblas on Linux.
* Fix Boost linker error (remove -mt suffix).
* use shadow-fixed
* add link to issues for each patchs
* update travis to check rosbuild/catkin, use_deb/use_source
* Contributors: Benjamin Chrétien, Kei Okada, Scott K Logan

3.1.5-5 (2014-03-04)
--------------------
* Fix to an issue that caused https://github.com/start-jsk/hrpsys/issues/25
* Initial commit of CHANGELOG.rst
* Contributors: Kei Okada, chen.jsk, Ryohei Ueda, Isaac Isao Saito, Hiroyuki Mikita, Iori Kumagai, Takuya Nakaoka, Shunichi Nozawa, Rosen Diankov, Yohei Kakiuchi
