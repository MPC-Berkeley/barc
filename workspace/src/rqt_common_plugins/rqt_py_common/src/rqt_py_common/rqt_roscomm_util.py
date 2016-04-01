# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac Saito

import os

import genmsg
import roslaunch
from roslaunch import RLException
import rospkg
import rospy
import rostopic


class RqtRoscommUtil(object):

    @staticmethod
    def load_parameters(config, caller_id):
        """
        Load parameters onto the parameter server.

        Copied from ROSLaunchRunner.

        @type config: roslaunch.config.ROSLaunchConfig
        @raise RLException:
        """

        # XMLRPC proxy for communicating with master, 'xmlrpclib.ServerProxy'
        param_server = config.master.get()

        param = None
        try:
            # multi-call style xmlrpc
            # According to API doc, get_multi() returns
            # multicall XMLRPC proxy for communicating with master,
            # "xmlrpclib.MultiCall"
            param_server_multi = config.master.get_multi()

            # clear specified parameter namespaces
            # #2468 unify clear params to prevent error
            for param in roslaunch.launch._unify_clear_params(
                                                          config.clear_params):
                if param_server.hasParam(caller_id, param)[2]:
                    param_server_multi.deleteParam(caller_id, param)
            r = param_server_multi()
            for code, msg, _ in r:
                if code != 1:
                    raise RLException("Failed to clear parameter {}: ".format(
                                                                         msg))
        except RLException:
            raise
        except Exception, e:
            rospy.logerr("load_parameters: unable to set params " +
                         "(last param was [{}]): {}".format(param, e))
            raise  # re-raise as this is fatal

        try:
            # multi-call objects are not reusable
            param_server_multi = config.master.get_multi()
            for param in config.params.itervalues():
                # suppressing this as it causes too much spam
                # printlog("setting parameter [%s]"%param.key)
                param_server_multi.setParam(caller_id, param.key, param.value)
            r = param_server_multi()
            for code, msg, _ in r:
                if code != 1:
                    raise RLException("Failed to set parameter: " +
                                                "%s" % (msg))
        except RLException:
            raise
        except Exception, e:
            print("load_parameters: unable to set params (last param was " +
                  "[%s]): %s" % (param, e))
            raise  # re-raise as this is fatal
        rospy.logdebug("... load_parameters complete")

    @staticmethod
    def iterate_packages(subdir):
        """
        Iterator for packages that contain the given subdir.

        This method is generalizing rosmsg.iterate_packages.

        @param subdir: eg. 'launch', 'msg', 'srv', 'action'
        @type subdir: str
        @raise ValueError:
        """
        if subdir == None or subdir == '':
            raise ValueError('Invalid package subdir = {}'.format(subdir))

        rospack = rospkg.RosPack()

        pkgs = rospack.list()
        rospy.logdebug('pkgs={}'.format(pkgs))
        for p in pkgs:
            d = os.path.join(rospack.get_path(p), subdir)
            rospy.logdebug('rospack dir={}'.format(d))
            if os.path.isdir(d):
                yield p, d

    @staticmethod
    def list_files(package, subdir, file_extension='.launch'):
        """
        #TODO: Come up with better name of the method.

        Taken from rosmsg.
        Lists files contained in the specified package

        @param package: package name, ``str``
        @param file_extension: Defaults to '.launch', ``str``
        :returns: list of msgs/srv in package, ``[str]``
        """
        if subdir == None or subdir == '':
            raise ValueError('Invalid package subdir = {}'.format(subdir))

        rospack = rospkg.RosPack()

        path = os.path.join(rospack.get_path(package), subdir)

        return [genmsg.resource_name(package, t)
                for t in RqtRoscommUtil._list_types(
                                                 path, file_extension)]

    @staticmethod
    def _list_types(path, ext):
        """
        Taken from rosmsg

        List all messages in the specified package
        :param package str: name of package to search
        :param include_depends bool: if True, will also list messages in
                                     package dependencies.
        :returns [str]: message type names
        """
        types = RqtRoscommUtil._list_resources(path,
                                               RqtRoscommUtil._msg_filter(ext))

        result = [x for x in types]
        # result = [x[:-len(ext)] for x in types]  # Remove extension

        result.sort()
        return result

    @staticmethod
    def _list_resources(path, rfilter=os.path.isfile):
        """
        Taken from rosmsg._list_resources

        List resources in a package directory within a particular
        subdirectory. This is useful for listing messages, services, etc...
        :param rfilter: resource filter function that returns true if filename
                        is the desired resource type, ``fn(filename)->bool``
        """
        resources = []
        if os.path.isdir(path):
            resources = [f for f
                         in os.listdir(path) if rfilter(os.path.join(path, f))]
        else:
            resources = []
        return resources

    @staticmethod
    def _msg_filter(ext):
        """
        Taken from rosmsg._msg_filter
        """
        def mfilter(f):
            """
            Predicate for filtering directory list. matches message files
            :param f: filename, ``str``
            """
            return os.path.isfile(f) and f.endswith(ext)
        return mfilter

    @staticmethod
    def is_roscore_running():
        """
        @rtype: bool
        """
        try:
            # Checkif rosmaster is running or not.
            rostopic.get_topic_class('/rosout')
            return True
        except rostopic.ROSTopicIOException as e:
            return False
