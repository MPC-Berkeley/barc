#!/usr/bin/env python

import os, sys
import time

os.chdir('/home/mpc/barc/workspace/src/data_service/scripts')

from data_connection import *
from base import *

proj_path = '/home/mpc/barc/Dator'

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'dator.settings')
sys.path.append(proj_path)
os.chdir(proj_path)

from django.core.wsgi import get_wsgi_application
application = get_wsgi_application()

from data_api.models import *

CLOUD_CONFIG_LOCATION = '/home/mpc/cloud.cfg'

if __name__ == '__main__':
    configurator = init_configurator(CLOUD_CONFIG_LOCATION)
    data_connection = DataConnection(configurator)

    for sig in Signal.objects.all():
        try:
            experiment = data_connection.get_or_create_experiment(sig.experiment.name)
            signal = data_connection.get_or_create_signal(sig.name, experiment)
            print signal

            try:
                lst = LocalSignalTag.objects.filter(signal__name=sig.name, signal__experiment__name=sig.experiment.name)[0]
            except (LocalSignalTag.DoesNotExist, IndexError) as e:
                lst = LocalSignalTag()
                lst.signal = sig
                lst.uploaded = False

            if not lst.uploaded:
                print 'Uploading'
                data_connection.add_signal_points(signal['id'], sig.get_data())
                lst.uploaded = True
                lst.save()
                print 'Finished Uploading'

            lst.save()

        except Exception as e:
            print e
