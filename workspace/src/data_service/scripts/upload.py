#!/usr/bin/env python

import os, sys
import time

os.chdir('/home/odroid/barc/workspace/src/data_service/scripts')

from data_connection import *
from base import *

proj_path = '/home/odroid/barc/Dator'

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'dator.settings')
sys.path.append(proj_path)
os.chdir(proj_path)

from django.core.wsgi import get_wsgi_application
application = get_wsgi_application()

from data_api.models import *

CLOUD_CONFIG_LOCATION = '/home/odroid/cloud.cfg'

if __name__ == '__main__':
    configurator = init_configurator(CLOUD_CONFIG_LOCATION)
    data_connection = DataConnection(configurator)

    for sig in Signal.objects.all():
        try:
            experiment = data_connection.get_or_create_experiment(sig.experiment.name)
            signal = data_connection.get_or_create_signal(sig.name, experiment)

            try:
                lst = LocalSignalTag.objects.get(signal=signal)
            except LocalSignalTag.DoesNotExist:
                lst = LocalSignalTag()
                lst.signal = signal
                lst.uploaded = False

            if not lst.uploaded:
                data_connection.add_signal_points(signal['id'], sig.get_data())
                lst.uploaded = True
                lst.save()

            lst.save()

        except Exception as e:
            print e
