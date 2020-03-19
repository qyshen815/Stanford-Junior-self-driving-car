'''
Created on Aug 6, 2010

@author: mvs
'''

import time
import sys
from LaunchCtrlProcess import *
import threading
import os
import signal
from collections import deque

class ArgumentException(Exception):
    
    def __init__(self, str):
        self.str = str

class LaunchCtrl():
    '''
    classdocs
    '''


    def __init__(self, args=None):
            '''
            Constructor
            '''
            if args:
                self.args = args
            else:
                self.args = list()
            self.processList = list()
    
    def load(self, filename):
            self.__loadFromFile(filename)
            
        
    def __loadFromFile(self, filename):
        file = open(filename, 'r')
        lines = file.readlines()
        file.close()
        # @todo: Make this more robust
        defaultpath = filename.split('/param/')[0] + '/bin'
        popenLock = threading.Lock()
        for line in lines:
            line = line.strip()
            if len(line) == 0:
                continue
            if line[0] == '#':
                continue
            tokens = line.split()
            if '@sleep' in tokens:
                if len(tokens) == 2:
                    if tokens[1].isdigit():
                        self.processList.append(LaunchCtrlSleep(int(tokens[1]))) 
                    else:
                        print "ctrl file has formatting error:", line
                        sys.exit(1)
                    continue
            output = True
            restart = False
            env = dict() 
            command = list()
            for item in tokens:
                if item == '@quiet':
                    output = False
                    continue
                if item == '@restart':
                    restart = True
                    continue
                if item.startswith('@host='):
                    env["CENTRALHOST"] = item[6:] + ";"
                    continue
                if item.startswith('@'):
                    print "unknown metacommand in ctrl file:", item
                    sys.exit(1)
                if item.startswith('$'):
                    if item[1:].isdigit():
                        num = int(item[1:])
                        if num > len(self.args):
                            raise ArgumentException("ctrl file requires more arguments than supplied!")
                        item = self.args[num - 1]
                command.append(item)
            command = " ".join(command)
            if command.startswith('/'):
                path = ""
            else:
                path = defaultpath
            
            self.processList.append(LaunchCtrlProcess(path, command, plock=popenLock, restart=restart, output=output, env=env))
            #addprocess(path, command, env=env, restart=restart, output=output)

    def start(self):
        for process in self.processList:
            process.start()
            
    def stop(self):
        for process in self.processList:
            process.stop()
            
    def kill(self):
        for process in self.processList:
            process.kill()


if __name__ == '__main__':

    launcher = None
    
    def exithandler(signum, frame):
        print "\nKilling Processes...\n"
        if launcher:
            launcher.kill()
        while threading.activeCount() != 1:
            pass
        time.sleep(1)
        sys.exit(0)
        
    signal.signal(signal.SIGINT, exithandler)
    
    # A real implementation for execution directory needs to be implemented
    path = os.getcwd()
    path = "".join( (path.split('/scripts')[0], "/bin" ) )
    
    strippedargs = deque()
    
    verbose = False
    logfile = False
    script = False
    
    if len(sys.argv) > 1 and (sys.argv[1] == "help" or sys.argv[1] == "--help"):
        print "LaunchCtrl usage:\n\n" \
            "\tArguments:\n" \
            "\t\t-v : Be verbose, otherwise no ouput from spawned processes will\n" \
            "\t\t     be printed\n" \
            "\t\t-l : Playback a logfile, first argument is the logfile\n" \
            "\t\t-s : Use a specific ctrl file, provided as the first argument\n" \
            "\n\tWith the exception of -l and -s flags, which consume the first argument,\n" \
            "\tall additional arguments are parsed and will be inserted into the\n" \
            "\tprocess file to replace $_1, $_2, etc....\n"
        sys.exit(0)
        
    for arg in sys.argv[1:]:
        if arg.startswith('-'):
            for char in arg:
                if char == 'v':
                    verbose = True
                if char == 'l':
                    logfile = True
                if char == 's':
                    script = True
        else:
            strippedargs.append(arg)
    
    if script and logfile:
        print "Error:\nCannot specify -l and -s at the same time"
        sys.exit(0)

    if logfile:
        file = "../../param/processes_playback.ctrl"
    elif script:
        file = strippedargs.popleft()
    else:
        file = "../../param/processes.ctrl"
    
    launcher = LaunchCtrl(args=strippedargs)
    try:
        launcher.load(file)
    except ArgumentException as e:
        print "Incorrect number of arguments for launch script"
        exithandler(0,0)
    
    launcher.start();
    
    def notNone(x):
        return x != None
    
    while True:
        (triggers, a, b) = select.select(filter(notNone,[proc.pty() for proc in launcher.processList]),[],[],0.1)
        if verbose:
            for active in triggers:
                while True:
                    try:
                        line = active.readline()
                    except IOError:
                        break
                    if line == "":
                        break
                    print line,
    
    
    
     
