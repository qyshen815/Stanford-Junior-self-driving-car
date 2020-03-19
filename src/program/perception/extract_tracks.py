#!/usr/bin/python

import sys
import os
import time
from subprocess import Popen

processes = []
try:
    race_root = os.getenv("RACE_ROOT")
    if(race_root is None):
        print "You must set RACE_ROOT!"
    assert(race_root is not None)
    assert(len(sys.argv) > 1 and len(sys.argv) < 4)
    
    bindir = race_root + "/bin"
    print "Using RACE_ROOT of " + race_root
    print "Binary dir: " + bindir


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
        
    if(len(sys.argv) > 2):
        cmd = bindir + "/perception " + sys.argv[2]
    else:
        cmd = bindir + "/perception " + os.path.basename(sys.argv[1][:-7])  + ".tm"
    extraction = Popen(["xterm", "-e", cmd])
    time.sleep(0.1);
    
    cmd = bindir + "/perception_view; bash"
    processes.append(Popen(["xterm", "-e", cmd]))
    time.sleep(1);

    os.system(bindir + "/dgc_playback_play 0.4")
    extraction.wait()



finally:
    print "\n"
    for p in processes:
        print "Killing ", p.pid
        p.kill()
    print



