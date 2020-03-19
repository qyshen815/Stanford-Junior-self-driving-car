'''
Created on Sep 2, 2010

@author: mvs
'''

import sys
import gzip
import dgc

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print "Usage:\r\t%s [logfilename]" % sys.argv[0]
        sys.exit(0)
    
    # Open logfile
    logfile = gzip.open(sys.argv[1])
    
    # Open KML output file
    kmlfile = open(sys.argv[1] + '.kml', 'w')
    
    # Write KML header
    kmlfile.write("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Path</name>
    <description>Examples of paths. Note that the tessellate tag is by default
      set to 0. If you want to create tessellated lines, they must be authored
      (or edited) directly in KML.</description>
    <Style id="redLineRedPoly">
      <LineStyle>
        <color>afff0000</color>
        <width>4</width>
      </LineStyle>
      <PolyStyle>
        <color>4fff0000</color>
      </PolyStyle>
    </Style>
    <Placemark>
      <name>Junior Path</name>
      <description>""" + sys.argv[1] + """</description>
      <styleUrl>#redLineRedPoly</styleUrl>
      <LineString>
        <extrude>1</extrude>
        <tessellate>0</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>""" + "\n")
    
    M_2_FT = 3.28084
    lasttime = 0
    try:
        for line in logfile:
            if line.startswith('#'):
                continue
            data = dgc.DGCMessage(line)
            if isinstance(data, dgc.APPLANIX_POSE_V2):
                if data.hardwaretime > lasttime + 0.2:
                    kmlfile.write("          " + str(data.gps[1]) + "," + str(data.gps[0]) + "," + str(int(data.gps[2] * M_2_FT)) + "\n")
                    lasttime = data.hardwaretime
    except IOError:
        print "Error reading from zipped logfile - output may be incomplete."
    
    # Write KML ender
    kmlfile.write("""        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>""")
        
    kmlfile.close()    
    logfile.close()
        