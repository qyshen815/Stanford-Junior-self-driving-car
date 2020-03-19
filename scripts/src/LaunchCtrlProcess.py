import time
import fcntl
import select
import subprocess
import pty
import threading
import signal
import os, sys
from collections import deque



class LaunchCtrlEntry():
    
    def start(self):
        pass
    
    def stop(self):
        pass

    def kill(self):
        pass
    
    def pty(self):
        return None

class LaunchCtrlSleep(LaunchCtrlEntry):
    
    def __init__(self, time):
        self.time = time
        
    def start(self):
        time.sleep(self.time)
    

class LaunchCtrlProcess(LaunchCtrlEntry):
    
    def __init__(self, path, command, plock=None, restart=False, output=True, env=None):
        """Arguments:
        
        path -- Working directory for the executable
        command -- Command line entry to execute, as a str
        plock -- Optional common Lock, if multiple Processes will be launched,
                 this should be provided to prevent multiple concurrent forks
        restart -- If true the process will be restarted if it exits for any
                   reason besides a stop or kill event
        output -- Should the process even bother reading it's output
        env -- A dict of options environment variables for the process
        """
        self._plock = plock
        self._stopevent = threading.Event()
        self._startevent = threading.Event()
        self._killevent = threading.Event()
        self._lock = threading.Lock();
        self._startCount = 0;
        self._running = False;
        self._pty = None
        #self._stdout = deque(maxlen=1000)
        self._returncodes = deque([])
        self._path = path
        self._command = command
        self._restart = restart
        self._output = output
        self._env = env
        
        self._process = LaunchCtrlThread(self)
        self._process.start()
        
    def start(self):
        self._lock.acquire()
        if not self._running and not self._startevent.isSet():
            self._startevent.set()
        self._lock.release()
        
    def stop(self):
        self._stopevent.set()
        
    def kill(self):
        self._killevent.set()
    
    def command(self):
        return self._command
    
    def startCount(self):
        self._lock.acquire()
        count = self._startCount
        self._lock.release()
        return count
    
    def running(self, value=None):
        self._lock.acquire()
        running = self._running
        self._lock.release()
        return running
    
    def stopping(self):
        if self._stopevent.isSet():
            return True
        else:
            return False
        
    def pty(self):
        self._lock.acquire()
        if isinstance(self._pty, int):
            try:
                self._pty = os.fdopen(self._pty, 'r')
                fd = self._pty.fileno()
                fl = fcntl.fcntl(fd, fcntl.F_GETFL)
                fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
            except:
                print "Could not attach to pty for reading"
                # @todo: do something sane here
                raise
        pty = self._pty
        self._lock.release()
        return pty
    
    def returncodes(self):
        self._lock.acquire()
        returncodes = self._returncodes
        self._lock.release()
        return returncodes    

class LaunchCtrlThread(threading.Thread):
    '''
    Seperate thread to manage an individual process.    Provides communication
    via a pty
    '''

    def __init__(self, parent):
        self.parent = parent
        self.parent._lock.acquire()
        self.startevent = parent._startevent
        self.stopevent = parent._stopevent
        self.killevent = parent._killevent
        self.path = parent._path
        self.env = parent._env
        self.command = parent._command.split()
        self.restart = parent._restart
        self.output = parent._output
        self.parent._lock.release()
        self.outwriter = None
        self.process = None

        if self.output:
            self.parent._lock.acquire()
            (self.parent._pty, slave) = pty.openpty()
            self.parent._lock.release()
            try:
                self.outwriter = os.fdopen(slave, 'w')
                fd = self.outwriter.fileno()
                fl = fcntl.fcntl(fd, fcntl.F_GETFL)
                fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
            except:
                print "Could not attach to pty for writing"
                # @todo: Do something sane here
                raise Exception
        threading.Thread.__init__(self)
            
    def run(self):
        # !!Rely on the cwd argument to Popen to take care of this
        #try:
        #    os.chdir(self.path)
        #except:
        #    print "Could not cd to", self.path, "Aborting!"
        #    raise
        if self.outwriter == None:
            self.outwriter = open(os.devnull)
        errfile = subprocess.STDOUT
        while True:
            try:
                select.select([], [], [], 0.1)
            except:
                pass
            
            # What to do if the process exists
            if self.process != None:
                self.process.poll()
                if self.process.returncode != None:
                    linestr = "Stopped (-) "+self.command[0]+"  ["+str(self.process.returncode)+"]\n"
                    print linestr,
                    self.parent._lock.acquire()
                    self.parent._returncodes.append(self.process.returncode)
                    self.parent._running = False
                    self.parent._lock.release()
                    self.process = None 
                    if self.restart == True and not self.stopevent.isSet():
                        self.startevent.set()
                    self.stopevent.clear()
                                     
         
            # What to do if we get a start process signal
            if self.startevent.isSet():
                if self.process == None:
                    try:
                        procenv = dict(os.environ.items() + self.env.items())
                        if self.parent._plock:
                            self.parent._plock.acquire()
                        self.process = subprocess.Popen(self.command, stderr=errfile,
                                stdout=self.outwriter, stdin=open(os.devnull), cwd=self.path,
                                preexec_fn=os.setsid, env=procenv)
                        if self.parent._plock:
                            self.parent._plock.release()
                        linestr = "Started (+) "+self.command[0]+"\n"
                        print linestr,
                        self.parent._lock.acquire()
                        self.parent._startCount += 1
                        self.parent._running = True
                        self.parent._lock.release()
                    except:
                        linestr = "Could not run binary", self.command[0], "Aborting!\n"
                        print linestr,
                        raise Exception
                self.startevent.clear()
         
            # What to do if we get a stop process signal
            if self.stopevent.isSet() or self.killevent.isSet():
                if self.process != None:
                    self.process.send_signal(signal.SIGINT)
                    count = 20
                    while count > 0:
                        try:
                            select.select([], [], [], 0.1)
                        except Exception, inst:
                            pass
                        self.process.poll()
                        if self.process.returncode == None:
                            count -= 1
                        else:
                            break
                    if count == 0:
                        linestr = "SIGKILL to process "+self.command[0]+"\n"
                        print linestr,
                        self.process.kill()
                    self.parent._lock.acquire()
                    self.parent._running = False
                    self.parent._lock.release()
                else:
                    self.stopevent.clear()
                if self.killevent.isSet():
                    linestr = "Killed "+self.command[0]+"\n"
                    print linestr,
                    return
                        


