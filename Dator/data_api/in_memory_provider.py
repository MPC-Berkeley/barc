import os
__author__ = 'brucewootton'
"""
In memory blob store with file lock and appending support.
"""

BLOBS = {}

def startup():
    pass


def file_name(uuid):
    pass


def get_blob(uuid):
    return BLOBS[uuid]


def write_blob(uuid, blob):
    BLOBS[uuid] = blob


def append_data(uuid, value):
    if uuid in BLOBS:
        BLOBS[uuid] += value
    else:
        BLOBS[uuid] = value


def clear(uuid):
    BLOBS.pop(uuid)



