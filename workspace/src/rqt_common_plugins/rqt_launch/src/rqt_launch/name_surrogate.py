#! /usr/bin/env python


class NamesSurrogate(object):
    '''
    Because some functions in roslib.names cannot be referred in the original
    rxlaunch code, the codes of those function are copied here. This class
    should not be used for any other purpose than to be used within this .py
    file.

    :author: Isaac Saito
    '''

    PRIV_NAME = '~'
    SEP = '/'

    @staticmethod
    def is_global(name):
        '''
        Test if name is a global graph resource name. 116 117
        @param name: must be a legal name in canonical form 118
        @type name: str 119
        @return: True if name is a globally referenced name (i.e. /ns/name) 120
        @rtype: bool
        '''
        return name and name[0] == NamesSurrogate.SEP

    @staticmethod
    def is_private(name):
        ''' 126 Test if name is a private graph resource name. 127 128
        @param name: must be a legal name in canonical form 129
        @type name: str 130 @return bool: True if name is a privately
                    referenced name (i.e. ~name) 131 '''
        return name and name[0] == NamesSurrogate.PRIV_NAME

    @staticmethod
    def ns_join(ns, name):
        '''
        Taken from
        http://ros.org/rosdoclite/groovy/api/roslib/html/python/roslib.names-pysrc.html#ns_join
        since roslib.names is not found for some reason, and also the entire
        module seems deprecated.

        Join a namespace and name. If name is unjoinable (i.e. ~private or
        162 /global) it will be returned without joining 163 164
        @param ns: namespace ('/' and '~' are both legal). If ns is the empty
        string, name will be returned. 165
        @type ns: str 166
        @param name str: a legal name 167
        @return str: name concatenated to ns, or name if it's 168 unjoinable. 169
        @rtype: str 170
        '''
        if NamesSurrogate.is_private(name) or NamesSurrogate.is_global(name):
            return name
        if ns == NamesSurrogate.PRIV_NAME:
            return NamesSurrogate.PRIV_NAME + name
        if not ns:
            return name
        if ns[-1] == NamesSurrogate.SEP:
            return ns + name
        return ns + NamesSurrogate.SEP + name
