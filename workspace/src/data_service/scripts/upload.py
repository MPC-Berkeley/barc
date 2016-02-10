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
            signal = data_connection.get_or_create_signal(sig.name)
            data_connection.add_signal_points(signal['id'], sig.get_data())
            sig.delete()
        except Exception as e:
            print e

