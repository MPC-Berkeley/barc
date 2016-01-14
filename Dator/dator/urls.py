"""ruenoor URL Configuration
"""
from django.conf.urls import include, url
from django.contrib import admin
from tastypie.api import Api
from data_api.api import *
from data_api.views import signal_data, blob_data, noop_view, claim_local_computer
from manage_ui.views import simple_view, root_view
#admin.autodiscover()

# tasty-pie definitions
v1_api = Api(api_name='v1')

# equipment resources
v1_api.register(SystemResource())
v1_api.register(ProgramResource())
v1_api.register(LocalComputerResource())
v1_api.register(CommandResource())
v1_api.register(SignalResource())
v1_api.register(SettingResource())
v1_api.register(EventResource())
v1_api.register(BlobResource())

urlpatterns = [
    url(r'^admin/', include(admin.site.urls)),
    url(r'^api/', include(v1_api.urls)),
    url(r'^dator/', simple_view, name='SimpleURL'),
    url(r'^data_api/v1/', include([
        url(r'^signal/(?P<signal_id>\d+)/', signal_data, name='signal_data'),
        url(r'^blob/(?P<blob_id>\d+)/', blob_data, name='blob_data'),
    ])),
    url(r'^claim_local_computer/(?P<local_computer_id>\d+)/', claim_local_computer, name='claim_local_computer'),
    url(r'^noop/', noop_view, name='NoopView'),
    url(r'^$', root_view, name='RootView')
]
