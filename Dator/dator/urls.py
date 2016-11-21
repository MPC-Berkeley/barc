"""ruenoor URL Configuration
"""
from django.conf.urls import include, url
from django.contrib import admin
from tastypie.api import Api
from data_api.api import *
from data_api.views import signal_data, blob_data, noop_view, claim_local_computer, clone_experiment, find_signals, experiment_media, setting_data
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
v1_api.register(ExperimentResource())


urlpatterns = [
    url(r'^admin/', include(admin.site.urls)),
    url(r'^api/v1/local_computer/', include([
        url(r'^(?P<local_computer_id>\d+)/experiment/(?P<source_experiment_id>\d+)/clone_experiment/',
            clone_experiment, name='CloneExperiment')
    ])),
    url(r'^api/', include(v1_api.urls)),
    url(r'^dator/', simple_view, name='SimpleURL'),
    url(r'^data_api/v1/', include([
        url(r'^local_computer/(?P<local_computer_id>\d+)/find_signals/', find_signals, name="FindSignals"),
        url(r'^signal/(?P<signal_id>\d+)/', signal_data, name='SignalData'),
        url(r'^blob/(?P<blob_id>\d+)/', blob_data, name='BlobData'),
        url(r'^experiment_media/(?P<experiment_id>\d+)/', experiment_media, name='ExperimentMedia'),
        url(r'^setting/(?P<setting_id>\d+)/', setting_data, name='SettingData'),
    ])),
    url(r'^claim_local_computer/(?P<local_computer_id>\d+)/', claim_local_computer, name='ClaimLocalComputer'),
    url(r'^noop/', noop_view, name='NoopView'),
    url(r'^$', root_view, name='RootView')
]
