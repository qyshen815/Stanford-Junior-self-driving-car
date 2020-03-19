#!/usr/bin/python

from BaseHTTPServer import HTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler
import os
import urllib
import pycurl

#The port of your local machine on which you want to run this web
#server.  You'll access the web server by visiting,
#e.g. "http://localhost:8000/"

PORT = 1234

class CurlContainer:
    def __init__(self):
        self.contents = ''

    def body_callback(self, buf):
        self.contents = self.contents + buf

class VisibleHTTPRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self, method='GET'):

        inurl = self.path.lstrip('/')
#        print "inurl = ", inurl

        inurlparts = inurl.rsplit('/', 1);
#        print "inurlparts = ", inurlparts
        if len(inurlparts) == 1:
            urlpath = ""
            urlfile = inurlparts[0]
        else:
            urlpath = inurlparts[0]
            urlfile = inurlparts[1]
#        print "urlpath =", urlpath
#        print "urlfile =", urlfile
        if not os.path.isfile(inurl):
            urlfilebase = os.path.splitext(urlfile)[0]
            components = urlfilebase.split('-')
            path_components = urlpath.split('/')
            x = 0
            while x < len(path_components):
                if path_components[x] == "":
                    del path_components[x]
                else:
                    x += 1
#            print "path_components =", path_components
            if (len(components) == 5 and components[0] == "usgs" and
                len(path_components) == 4):
                terra_type = int(path_components[1])
                terra_res = int(components[1])
                terra_x = int(components[2])
                terra_y = int(components[3])
                terra_zone = int(components[4])

                params = urllib.urlencode({'T': terra_type, 'S': terra_res,
                                           'X': terra_x, 'Y': terra_y,
                                           'Z': terra_zone})
                terraurl = "http://terraserver-usa.com/tile.ashx?%s" % params

                t = CurlContainer()
                c.setopt(c.URL, terraurl);
                c.setopt(c.WRITEFUNCTION, t.body_callback)
                c.perform()

                self.wfile.write(t.contents)

                newdir = "usgs/%d/%d/%d" % (terra_type, terra_res, terra_x)
                if not os.path.isdir(newdir):
                    os.makedirs(newdir)
                newfilename = "%s/usgs-%d-%d-%d-%d.jpg" % (newdir, terra_res,
                                                           terra_x, terra_y,
                                                           terra_zone)
                print "saving file", newfilename
                sample_file = file(newfilename, "w")
                sample_file.write(t.contents)
                del sample_file
                del t
            else:
                self.send_error(404)
        else:
            sample_file = file(inurl, "r")
            self.copyfile(sample_file, self.wfile)
            del sample_file

if __name__ == '__main__':
    httpd = HTTPServer(('driving.stanford.edu', PORT),
                       VisibleHTTPRequestHandler)
    c = pycurl.Curl()
    httpd.serve_forever()
