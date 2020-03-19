#!/usr/bin/python

import sys
import os
import time
from subprocess import Popen

for arg in sys.argv:
    print arg
    
if len(sys.argv) != 2:
    print "Usage: dgc_launch.py LOG.GZ"
    sys.exit(0)

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
    processes.append(Popen(["xterm", "-e", cmd]))
    time.sleep(0.1);
    
    cmd = bindir + "/dgc_playback_control"
    processes.append(Popen([cmd]))
    time.sleep(0.1);

    # -- perception with track extraction
#    cmd = bindir + "/perception " + os.path.basename(sys.argv[1][:-7])  + ".tm; bash"
#    environ = os.environ;
#    #environ["DISABLE_RNDF_FILTER"] = "";
#    environ["EXTRACT_TRACKS"] = "";
#    processes.append(Popen(["xterm", "-e", cmd]))
#    time.sleep(0.1);

    # -- regular perception
    cmd = bindir + "/perception; bash"
    processes.append(Popen(["xterm", "-e", cmd]))
    time.sleep(0.1);
    
    cmd = bindir + "/perception_view; bash"
    processes.append(Popen(["xterm", "-e", cmd]))
    time.sleep(0.1);
    
    while(True):
        time.sleep(100)

finally:
    print "\n"
    for p in processes:
        print "Killing ", p.pid
        p.kill()
    print



