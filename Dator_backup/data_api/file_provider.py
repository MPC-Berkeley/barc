import os
from pip._vendor.lockfile import LockFile

__author__ = 'brucewootton'
"""
Simple blob store with file lock and appending support.
"""

DATA_DIR = "/home/odroid/data_rep".format(os.path.dirname(os.path.abspath(__file__)))

def startup():
    if not os.path.isdir(DATA_DIR):
        os.mkdir(DATA_DIR)

def file_name(uuid):
    dir_name = "{}/{}".format(DATA_DIR, uuid[0:5])
    if not os.path.isdir(dir_name):
        os.mkdir(dir_name)
    return "{}/{}.fb".format(dir_name, uuid)

def get_blob(uuid):
    """
    :param uuid:
    :return: return the blob corresponding to the file id
    """
    lock = LockFile(file_name(uuid), timeout=10)
    with lock:
        with open(file_name(uuid), "r") as input:
            return input.read()

def write_blob(uuid, blob):
    lock = LockFile(file_name(uuid), timeout=10)
    with lock:
        with open(file_name(uuid), "w+") as input:
            input.write(blob)

def append_data(uuid, value):
    lock = LockFile(file_name(uuid)+".lock", timeout=10)
    with lock:
        with open(file_name(uuid), "a") as output:
            output.write(value)

def clear(uuid):
    lock = LockFile(file_name(uuid), timeout=10)
    with lock:
        os.remove(file_name(uuid))



