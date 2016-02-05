"""
WSGI config for ruenoor project.

It exposes the WSGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/1.8/howto/deployment/wsgi/
"""

import os
import sys
from django.core.wsgi import get_wsgi_application

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "dator.settings")
path = '/home/ubuntu/dator'
if path not in sys.path:
    sys.path.append(path)
sys.path.append('/home/ubuntu/dator/dator')

sys.path.append('/home/ubuntu/dist/lib/python2.7/site-packages')

application = get_wsgi_application()
