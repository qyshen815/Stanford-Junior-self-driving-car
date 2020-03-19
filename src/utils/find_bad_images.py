#!/usr/bin/python

import os

def search_tree(dir_path):
    for name in os.listdir(dir_path):
        full_path = os.path.join(dir_path, name)
        if os.path.isfile(full_path) and full_path.endswith(".jpg"):
            fp = file(full_path, "r")
            image = fp.read()
            del fp

            if(len(image) < 8 or ord(image[0]) != 0xFF or
               ord(image[1]) != 0xD8 or ord(image[2]) != 0xFF or
               ord(image[3]) != 0xE0):
                print full_path
        if os.path.isdir(full_path):
            search_tree(full_path)

search_tree('usgs')
