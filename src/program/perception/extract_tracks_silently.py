#!/usr/bin/python

import sys
import os
import time
from subprocess import Popen

processes = []
FNULL = open('/dev/null', 'w')
try:
    race_root = os.getenv("RACE_ROOT")
    if(race_root is None):
        print "You must set RACE_ROOT!"
        sys.exit()

    if(not (len(sys.argv) > 1 and len(sys.argv) < 4)):
        print "You must give one or two arguments."
        sys.exit()

    if(not os.path.exists(sys.argv[1])):
        print "Cannot find " + sys.argv[1]
        sys.exit()
    
    bindir = race_root + "/bin"
    print "Using RACE_ROOT of " + race_root
    print "Binary dir: " + bindir

    processes.append(Popen(bindir + "/central", stdout=FNULL, stderr=FNULL))
    time.sleep(0.5);
    
    path = race_root + "/param/roadrunner.ini"
    cmd = bindir + "/param_server"
    environ = os.environ;
    environ["RACE_ROOT"] = race_root;
    processes.append(Popen([cmd, path], env=environ, stdout=FNULL, stderr=FNULL))
    time.sleep(0.5);
    
    cmd = bindir + "/dgc_playback"
    print cmd
    processes.append(Popen([cmd, sys.argv[1]], stdout=FNULL, stderr=FNULL))
    time.sleep(0.1);

    cmd = bindir + "/perception"
    if(len(sys.argv) > 2):
        arg = sys.argv[2]
    else:
        arg = os.path.basename(sys.argv[1][:-7]) + ".tm"
    extraction = Popen([cmd, arg], stderr=FNULL)
    time.sleep(1);
    
    os.system(bindir + "/dgc_playback_play 0.25")
    extraction.wait()

finally:
    print "\n"
    for p in processes:
        print "Killing ", p.pid
        p.kill()
    print



