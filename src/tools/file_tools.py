# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# ----- the common modules
import os
from pathlib import Path

# ----- file utilities
# -- returns the filename of a path (the last part of the path)
def get_filename(path):
    return Path(path).name

# -- returns the suffix of a path or a filename
def get_suffix(path, include_period=True):
    suffix = Path(path).suffix
    if not include_period and len(suffix) > 0:
        return suffix[1:]
    return suffix

# -- replace suffix with another
def replace_suffix(path, new_suffix):
    index = path.rfind('.')
    if index == -1:
        return f'{path}.{new_suffix}'
    return f'{path[:index]}.{new_suffix}' 

# -- returns the parent folder of a file (path)
def get_parent(path):
    return os.path.realpath(Path(path).parent)

# -- move the file from source to dest
def move_file(source, dest):
    Path(source).rename(dest)

# -- returns a list of files found in the folder with a suffix (if specified)
# -- the list contains full path of the files if full_path is True
def list_files(folder, full_path=False, suffix=None):
    if suffix is None:
        filelist = [f for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]
    else:
        filelist = [f for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f)) and f.endswith(suffix)]
    if full_path:
        file_list = list(map(lambda f: os.path.join(folder, f), file_list))    
    return filelist

# -- check if child is parent or its subdirectory
def is_subdir(child, parent):
    if os.path.isdir(child) and os.path.isdir(parent):
        child, parent = os.path.realpath(child), os.path.realpath(parent)
        return child == parent or child.startswith(parent + os.sep)
    else:
        return False

# -- prepend the parent path to the front of the list of files in file_list
def prepend_path(file_list, parent):
    file_list = map(lambda f: os.path.join(parent, f), file_list)
    return list(file_list)

# -- return (one of the) file name starting with a given string in a folder
def search_filename_startswith(folder, prefix=None):
    for f in os.listdir(folder):
        if prefix is None or f.startswith(prefix):
            return f
    return None

# -- return the size of a folder
def get_folder_size(folder):
    total = 0
    for path, dirs, files in os.walk(folder):
        for f in files:
            fp = os.path.join(path, f)
            total += os.path.getsize(fp)
    return total
    
# -- the main (test) program
if __name__ == '__main__':
    pass