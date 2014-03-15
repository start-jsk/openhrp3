#!/usr/bin/env python

PKG = 'openhrp3'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import os
import sys
import unittest

code = """
#include <hrpModel/Body.h>

int main (int argc, char** argv)
{
  hrp::BodyPtr body(new hrp::Body());
  return 0;
}
"""
from subprocess import call, check_output, Popen, PIPE, STDOUT

## A sample python unit test
class TestCompile(unittest.TestCase):
    PKG_CONFIG_PATH = ''

    def setUp(self):
        # if rosbuild environment
        openhrp3_path = check_output(['rospack','find','openhrp3']).rstrip()
        if os.path.exists(os.path.join(openhrp3_path, "bin")) :
            self.PKG_CONFIG_PATH='PKG_CONFIG_PATH=%s/lib/pkgconfig:$PKG_CONFIG_PATH'%(openhrp3_path)

    ## test 1 == 1
    def test_compile_pkg_config(self):
        global PID
        cmd = "%s pkg-config openhrp3.1 --cflags --libs"%(self.PKG_CONFIG_PATH)
        print "`"+cmd+"` =",check_output(cmd, shell=True, stderr=STDOUT)
        ret = call("gcc -o openhrp3-sample-pkg-config /tmp/%d-openhrp3-sample.cpp `%s`"%(PID,cmd), shell=True)
        self.assertTrue(ret==0)

    def _test_compile_move_ankle(self):
        cmd1 = "pkg-config openhrp3.1 --cflags --libs"
        cmd2 = "pkg-config openhrp3.1 --variable=idl_dir"
        print "`"+cmd1+"` =",check_output(cmd1, shell=True, stderr=STDOUT)
        print "`"+cmd2+"` =",check_output(cmd2, shell=True, stderr=STDOUT)
        ret = call("gcc -o move_ankle `%s`/../sample/example/move_ankle/move_ankle.cpp `%s`"%(cmd2,cmd1), shell=True)
        self.assertTrue(ret==0)

    def test_idl_dir(self):
        cmd = "%s pkg-config openhrp3.1 --variable=idl_dir"%(self.PKG_CONFIG_PATH)
        fname = "OpenHRP/OpenHRPCommon.idl"
        # check if dil file exists
        print "`"+cmd+"`"+fname+" = "+os.path.join(check_output(cmd, shell=True).rstrip(), fname)
        self.assertTrue(os.path.exists(os.path.join(check_output(cmd, shell=True).rstrip(), fname)))

    def test_sample_pa10(self):
        cmd = "%s pkg-config openhrp3.1 --variable=idl_dir"%(self.PKG_CONFIG_PATH)
        fname = "../sample/model/PA10/pa10.main.wrl"
        # check if dil file exists
        print "`"+cmd+"`"+fname+" = "+os.path.join(check_output(cmd, shell=True).rstrip(), fname)
        self.assertTrue(os.path.exists(os.path.join(check_output(cmd, shell=True).rstrip(), fname)))

#unittest.main()
if __name__ == '__main__':
    import rostest
    global PID
    PID = os.getpid()
    f = open("/tmp/%d-openhrp3-sample.cpp"%(PID),'w')
    f.write(code)
    f.close()
    rostest.rosrun(PKG, 'test_openhrp3', TestCompile) 



