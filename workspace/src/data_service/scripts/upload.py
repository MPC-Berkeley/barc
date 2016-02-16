#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

import os, sys
import time

os.chdir( os.path.expanduser("~") + '/barc/workspace/src/data_service/scripts')

from data_connection import *
from base import *

proj_path = os.path.expanduser("~") + '/barc/Dator'

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'dator.settings')
sys.path.append(proj_path)
os.chdir(proj_path)

from django.core.wsgi import get_wsgi_application
application = get_wsgi_application()

from data_api.models import *

import boto3

CLOUD_CONFIG_LOCATION = os.path.expanduser("~") + '/cloud.cfg'


S3_VIDEOS_BUCKET = 'datorvideos2'

rosbag_dir = os.path.expanduser("~") + '/rosbag'
video_dir = os.path.expanduser("~") + '/video'


if __name__ == '__main__':
    configurator = init_configurator(CLOUD_CONFIG_LOCATION)
    data_connection = DataConnection(configurator)

    s3_client = boto3.client('s3', use_ssl=0)
    s3 = boto3.resource('s3', use_ssl=0)

    for sig in Signal.objects.all():
        try:
            experiment = data_connection.get_or_create_experiment(sig.experiment.name)
            print 'Looking at experiment : %s ' % experiment['name']

            for setting in sig.experiment.setting_set.all():
                setting_remote = data_connection.get_or_create_setting(setting.key, experiment)
                print 'Looking at signal : %s ' % sig.name

                #TODO: Check if AWS S3 token exists
                if setting.key == 'video':
                    if setting.value.startswith(video_dir):
                        key_name = '%s_%s.avi' % (os.environ['TEAM_NAME'], sig.experiment.name)
                        video_path = '%s/%s.avi' %(video_dir, sig.experiment.name)

                        if os.path.isfile(video_path):
                           bucket = s3.Bucket(S3_VIDEOS_BUCKET)
                           bucket.Acl().put(ACL='public-read')

                           obj = s3.Object(S3_VIDEOS_BUCKET, key_name)
                           print 'Uploading video : %s ' % sig.experiment.name
                           obj.put(Body=open(video_path, 'rb'))
                           obj.Acl().put(ACL='public-read')

                           print 'Finished uploading video'
                           url = '{}/{}/{}'.format(s3_client.meta.endpoint_url,
                                                     S3_VIDEOS_BUCKET, key_name)
                           setting.value = url
                           setting.save()
                           os.remove(video_path)
                        else:
                           print 'WARNING: Video no longer available. You will have an unlinked video in your S3 storage at:'
                           print setting.value
                           setting.value = ''
                           setting.save()


                data_connection.write_setting(setting.value, setting_remote['id'])

            signal = data_connection.get_or_create_signal(sig.name, experiment)
            try:
                lst = LocalSignalTag.objects.filter(signal__name=sig.name, signal__experiment__name=sig.experiment.name)[0]
            except (LocalSignalTag.DoesNotExist, IndexError) as e:
                lst = LocalSignalTag()
                lst.signal = sig
                lst.uploaded = False

            if not lst.uploaded:
                print 'Uploading ...'
                try:
                   data_connection.add_signal_points(signal['id'], sig.get_data())
                   lst.uploaded = True
                   lst.save()
                   print 'Finished Uploading'
                   sig.delete()
                   print 'Signal deleted locally'
                except Exception as e:
                    print 'Uploading signal failed...'
                    print e
            else:
                sig.delete()
                print 'Signal already stored on the web... deleting it locally'
    

            lst.save()

        except Exception as e:
            print e
