#!/usr/bin/env python

try: # catkin does not requires load_manifest
    import openhrp3
except:
    import roslib; roslib.load_manifest("openhrp3")

#import OpenRTM_aist.RTM_IDL # for catkin

from omniORB import CORBA, any, cdrUnmarshal, cdrMarshal
import CosNaming

import sys, os, socket

from OpenHRP import *
#improt OpenHRP
#import hrpsys
#import rtm

import rospkg

rootrc = None
# set custom port for modelloader-test.launch
def initCORBA():
    global rootnc
    nshost = socket.gethostname()
    nsport = 15005

    # initCORBA
    os.environ['ORBInitRef'] = 'NameService=corbaloc:iiop:{0}:{1}/NameService'.format(nshost,nsport)
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

    nameserver = orb.resolve_initial_references("NameService");
    rootnc = nameserver._narrow(CosNaming.NamingContext)

def findModelLoader():
    global rootnc
    try:
        obj = rootnc.resolve([CosNaming.NameComponent('ModelLoader', '')])
        return obj._narrow(ModelLoader_idl._0_OpenHRP__POA.ModelLoader)
    except:
        print "Could not find ModelLoader", sys.exc_info()[0]
        exit

def equal(a, b, tol = 1e-4):
    if type(a) == float and type(b) == float:
        return abs(a - b) < tol
    if type(a) == list and type(b) == list:
        return all(equal(x,y) for (x,y) in zip(a,b))
    else:
        return True

def norm(a):
    r = 0
    for e in a:
        r = r + e*e
    return r/len(a)

def print_ok(fmt, ok):
    s = '\033[0m' if ok else '\033[91m'
    e = '\033[0m'
    print s+"{0:70} {1}".format(fmt, ok)+e

def checkModels(wrl_file, dae_file):
    wrl_url = rospkg.RosPack().get_path("openhrp3")+"/share/OpenHRP-3.1/sample/model/"+wrl_file
    dae_url = rospkg.RosPack().get_path("openhrp3")+"/share/OpenHRP-3.1/sample/model/"+dae_file
    wrl_binfo = ml.getBodyInfo(wrl_url)
    dae_binfo = ml.getBodyInfo(dae_url)
    wrl_links = wrl_binfo._get_links()
    dae_links = dae_binfo._get_links()
    ret = True
    print "%16s %16s"%(wrl_file, dae_file)
    for (wrl_l, dae_l) in zip(wrl_links, dae_links) :
        # 'centerOfMass', 'childIndices', 'climit', 'encoderPulse', 'gearRatio', 'hwcs', 'inertia', 'inlinedShapeTransformMatrices', 'jointAxis', 'jointId', 'jointType', 'jointValue', 'lights', 'llimit', 'lvlimit', 'mass', 'name', 'parentIndex', 'rotation', 'rotorInertia', 'rotorResistor', 'segments', 'sensors', 'shapeIndices', 'specFiles', 'torqueConst', 'translation', 'ulimit', 'uvlimit'
        name_ok             = wrl_l.name == dae_l.name
        translation_ok      = equal(wrl_l.translation, dae_l.translation)
        rotation_ok         = equal(norm(wrl_l.rotation), norm(dae_l.rotation))
        mass_ok             = equal(wrl_l.mass, dae_l.mass)
        centerOfMass_ok     = equal(wrl_l.centerOfMass, dae_l.centerOfMass)
        inertia_ok          = equal(wrl_l.inertia, dae_l.inertia)
        llimit_ok           = equal(wrl_l.llimit, dae_l.llimit)
        ulimit_ok           = equal(wrl_l.ulimit, dae_l.ulimit)
        lvlimit_ok          = equal(wrl_l.lvlimit, dae_l.lvlimit)
        uvlimit_ok          = equal(wrl_l.uvlimit, dae_l.uvlimit)
        ret = all([ret, name_ok, translation_ok, rotation_ok, mass_ok, centerOfMass_ok])
        print_ok("name   {0:24s}  {1:24s} ".format(wrl_l.name, dae_l.name), name_ok)
        print_ok(" tran   {0:24}  {1:24}".format(wrl_l.translation, dae_l.translation), translation_ok)
        print_ok(" rot    {0:24}  {1:24}".format(wrl_l.rotation, dae_l.rotation), rotation_ok)
        print_ok(" mass   {0:<24}  {1:<24}".format(wrl_l.mass, dae_l.mass), mass_ok)
        print_ok(" CoM    {0:24}  {1:24}".format(wrl_l.centerOfMass, dae_l.centerOfMass), centerOfMass_ok)
        print_ok(" iner   {0:50}\n        {1:50}".format(wrl_l.inertia, dae_l.inertia), inertia_ok)
        print_ok(" llim   {0:24}  {1:24}".format(wrl_l.llimit, dae_l.llimit), llimit_ok)
        print_ok(" ulim   {0:24}  {1:24}".format(wrl_l.ulimit, dae_l.ulimit), ulimit_ok)
        print_ok(" lvlim  {0:24}  {1:24}".format(wrl_l.lvlimit, dae_l.lvlimit), lvlimit_ok)
        print_ok(" uvlim  {0:24}  {1:24}".format(wrl_l.uvlimit, dae_l.uvlimit), uvlimit_ok)
        for (wrl_s, dae_s) in zip(wrl_l.segments, dae_l.segments):
            # name, mass, centerOfMass, inertia, transformMatrix, shapeIndices
            name_ok             = wrl_s.name == dae_s.name
            mass_ok             = equal(wrl_s.mass, dae_s.mass)
            centerOfMass_ok     = equal(wrl_s.centerOfMass, dae_s.centerOfMass)
            inertia_ok          = equal(wrl_s.inertia, dae_s.inertia)
            transformMatrix_ok  = equal(wrl_s.transformMatrix, dae_s.transformMatrix)
            shapeIndices_ok     = equal(wrl_s.shapeIndices, dae_s.shapeIndices)
            ret = all([ret, name_ok, mass_ok, centerOfMass_ok,inertia_ok, transformMatrix_ok, shapeIndices_ok])
            print_ok(" name {0:24s}  {1:24s} ".format(wrl_s.name, dae_s.name), name_ok)
            print_ok("  mass  {0:<24}  {1:<24}".format(wrl_s.mass, dae_s.mass), mass_ok)
            print_ok("  CoM   {0:24}  {1:24}".format(wrl_s.centerOfMass, dae_s.centerOfMass), centerOfMass_ok)
            print_ok("  iner  {0:50}\n        {1:50}".format(wrl_s.inertia, dae_s.inertia), inertia_ok)
            print_ok("  trans {0:50}\n        {1:50}".format(wrl_s.transformMatrix, dae_s.transformMatrix), transformMatrix_ok)
            print_ok("  shape  {0:24}  {1:24}".format(wrl_s.shapeIndices, dae_s.shapeIndices), shapeIndices_ok)

    if not ret:
        print "===========\n== ERROR == {0} and {1} differs\n===========".format(wrl_file, dae_file)
    return ret

initCORBA()
ml = findModelLoader()
ret1 = checkModels("sample.wrl","sample.dae")
ret2 = checkModels("PA10/pa10.main.wrl","PA10/pa10.dae")
if not ret1 or not ret2:
    exit(1)
