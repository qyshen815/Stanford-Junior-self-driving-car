'''
Created on Nov 23, 2009

@author: mvs
'''

import dgc
import gzip

class log:
    '''
    classdocs
    '''
    message = None
    line = ''
    time = -1.0
    
    def __init__(self, logfilename):
        '''
        Opens the specified dgc log file for reading
        '''
        if logfilename.endswith('gz'):
            self.file = gzip.open(logfilename, 'r')
        else:
            self.file = open(logfilename, 'r')
        
    def nextmessage(self):
        '''
        Loads and returns the next non-comment line in the log file
        Returns an empty string if EOF has been reached
        '''
        self.line = self.file.readline()
        while self.line.startswith('#'):
            self.line = self.file.readline()
        if self.line == '':
            self.message = None
            self.time = -1.0
            return self.line
        self.message = dgc.DGCMessage(self.line)
        self.time = self.message.logtime
        return self.line
        
            
        