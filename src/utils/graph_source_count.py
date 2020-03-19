#!/usr/bin/python

from datetime import date
from datetime import timedelta
import os
import sys
import subprocess as sub
import fnmatch
from os.path import join, getsize

def build_wc_cmdline(startdir):
    cmdline = ['wc', '-l']
    for root, dirs, files in os.walk(startdir):
        for name in files:
            if fnmatch.fnmatch(name, '*.cpp') or fnmatch.fnmatch(name, '*.c') or fnmatch.fnmatch(name, '*.h'):
                cmdline.append(join(root, name))
        for name in ['.svn', 'development', 'branches', 'tags', 'ext', 'include', 'vehiclemodels', 'passatmodel', 'touaregmodel']:
            if name in dirs:
                dirs.remove(name)
    return cmdline

# comment out following 2 lines if you want to run this program
print "This program executes SVN commands. Comment out appropriate lines if you really want to run this."
sys.exit(1)

d = date(2006, 04, 07)

delta = timedelta(1)

f=open('/tmp/linedata.txt', 'a')

while d < date(2008, 5, 12):
    print 'Checking out version from ', d
    
    # checkout the repository 
    str = "svn update -q -r '{" + d.strftime("%Y-%m-%d") + "}' driving"
    os.system(str)

    # build the word counting command line
    line = build_wc_cmdline('.')
    if len(line) <= 2:
        f.write(d.strftime("%Y-%m-%d") + ' 0\n');
        f.flush()
    else:
        # run the code counter and get the output 
        p = sub.Popen(line, stdout=sub.PIPE, stderr=sub.PIPE)
        output, errors = p.communicate()
        # just save the last line of the output
        for outputline in output.splitlines():
            lastline = outputline
        f.write(d.strftime("%Y-%m-%d") + ' ' + lastline.split()[0] + '\n')
        f.flush()
    d += delta
f.close()


