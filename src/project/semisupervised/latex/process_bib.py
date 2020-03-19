#!/usr/bin/python

import os
import sys
import string

def process(entry, url):

    lines = string.split(entry, '\n')
    kill_annote = False
    for idx, line in enumerate(lines):
        if kill_annote:
            pass
#            if line[-2:] == '},':
#                kill_annote = False
#            lines[idx] = ''
        elif url is not '' and line[0:7] == 'title =':
            lines[idx] = line[0:9] + '\href' + url + line[9:-1] + ','
#        elif line[0:8] == 'annote =':
#            lines[idx] = 'annote = {},'
#            kill_annote = True
            
    return string.join(lines, '\n')
    

if(len(sys.argv) != 2):
    print 'Usage: python process_bib.py BIBFILE'
    sys.exit(1)

bibpath = sys.argv[1]
newbib = ''
newentry = ''
url = ''
    
f = open(bibpath)
while True:
    line = f.readline()
    if not line:
        break

    if line[0] == '@':
        newbib += process(newentry, url)
        newentry = line
        url = ''
    else:
        newentry += line
        if(line[0:5] == 'url ='):
            url = line[6:-2]

newbib += process(newentry, url)
print newbib
