#!/usr/bin/python

import sys
import os
import time
from subprocess import Popen

try:
    race_root = os.getenv("RACE_ROOT")
    if(race_root is None):
        cwd = os.getcwd()
        race_root = cwd.replace("/src/program/perception", "")

    bindir = race_root + "/bin"
    print "Using RACE_ROOT of " + race_root
    print "Binary dir: " + bindir

    processes = []

    processes.append(Popen(["xterm", "-e", bindir + "/central"]))
    time.sleep(0.5);
    
    path = race_root + "/param/roadrunner.ini"
    cmd = bindir + "/param_server"
    environ = os.environ;
    environ["RACE_ROOT"] = race_root;
    processes.append(Popen([cmd, path], env=environ))
    time.sleep(0.5);
   
    cmd = bindir + "/dgc_playback " + sys.argv[1] + "; bash"
    print cmd
    processes.append(Popen(["xterm", "-e", cmd]))
    time.sleep(0.1);
    
    cmd = bindir + "/dgc_playback_control"
    print cmd
    processes.append(Popen([cmd]))
    time.sleep(0.1);
    
    cmd = bindir + "/perception_view; bash"
    print cmd
    processes.append(Popen(["xterm", "-e", cmd]))
    time.sleep(0.2);

    cmd = bindir + "/ladybug_view"
    print cmd
    environ = os.environ;
    processes.append(Popen([cmd], env=environ))
    time.sleep(0.1);
    
    while(True):
        time.sleep(100)

finally:
    print "\n"
    for p in processes:
        print "Killing ", p.pid
        p.kill()
    print
    
    os.system("killall -9 dgc_playback")



