import json
from tastypie.authentication import Authentication
from tastypie.fields import IntegerField, DateTimeField, CharField, BooleanField
from tastypie.resources import ModelResource
from tastypie.authorization import Authorization
from tastypie.resources import ALL_WITH_RELATIONS
from tastypie import fields
from data_api.models import System, Program, Command, LocalComputer, Signal, Setting, Event, Blob, Experiment

from django.core.serializers.json import DjangoJSONEncoder
from tastypie.serializers import Serializer

class PrettyJSONSerializer(Serializer):
    json_indent = 2

    def to_json(self, data, options=None):
        options = options or {}
        data = self.to_simple(data, options)
        return json.dumps(data, cls=DjangoJSONEncoder,
                sort_keys=True, ensure_ascii=False, indent=self.json_indent)

class SystemResource(ModelResource):

    class Meta:
        queryset = System.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'system'
        always_return_data = True

        serializer = PrettyJSONSerializer()

class ProgramResource(ModelResource):

    class Meta:
        queryset = Program.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'program'
        always_return_data = True

        serializer = PrettyJSONSerializer()

class CommandResource(ModelResource):
    local_computer_id = IntegerField(attribute="local_computer_id")
    is_executed = BooleanField(attribute="is_executed")

    class Meta:
        queryset = Command.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'command'
        always_return_data = True
        filtering = {
            'local_computer_id': ALL_WITH_RELATIONS,
            'is_executed': ALL_WITH_RELATIONS
        }

        serializer = PrettyJSONSerializer()

class LocalComputerResource(ModelResource):

    name = CharField(attribute="name", null=True)

    class Meta:
        queryset = LocalComputer.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'local_computer'
        always_return_data = True

        serializer = PrettyJSONSerializer()

        filtering = {
            'name':  ALL_WITH_RELATIONS
        }

class SignalResource(ModelResource):
    local_computer_id = IntegerField(attribute="local_computer_id")
    system_id = IntegerField(attribute="system_id", null=True)
    name = CharField(attribute="name", null=True)
    experiment_id = IntegerField(attribute="experiment_id", null=True)

    class Meta:
        queryset = Signal.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'signal'
        always_return_data = True

        filtering = {
            'local_computer_id': ALL_WITH_RELATIONS,
            'name': ALL_WITH_RELATIONS,
            'experiment_id': ALL_WITH_RELATIONS
        }

        serializer = PrettyJSONSerializer()


class BlobResource(ModelResource):
    local_computer_id = IntegerField(attribute="local_computer_id")
    system_id = IntegerField(attribute="system_id", null=True)
    name = CharField(attribute="name", null=True)
    experiment_id = IntegerField(attribute="experiment_id", null=True)

    class Meta:
        queryset = Blob.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'blob'
        always_return_data = True

        filtering = {
            'local_computer_id': ALL_WITH_RELATIONS,
            'system_id': ALL_WITH_RELATIONS,
            'name': ALL_WITH_RELATIONS,
            'experiment_id': ALL_WITH_RELATIONS,
            'mime_type': ALL_WITH_RELATIONS
        }

        serializer = PrettyJSONSerializer()

class SettingResource(ModelResource):
    local_computer_id = IntegerField(attribute="local_computer_id")
    system_id = IntegerField(attribute="system_id", null=True)
    experiment_id = IntegerField(attribute="experiment_id", null=True)

    class Meta:
        queryset = Setting.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'setting'
        always_return_data = True

        filtering = {
            'local_computer_id': ALL_WITH_RELATIONS,
            'system_id': ALL_WITH_RELATIONS,
            'key': ALL_WITH_RELATIONS,
            'experiment_id': ALL_WITH_RELATIONS,
            'value': ALL_WITH_RELATIONS
        }

        serializer = PrettyJSONSerializer()


class EventResource(ModelResource):
    local_computer_id = IntegerField(attribute="local_computer_id")
    system_id = IntegerField(attribute="system_id", null=True)
    experiment_id = IntegerField(attribute="experiment_id", null=True)

    class Meta:
        queryset = Event.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'event'
        always_return_data = True
        max_limit=0
        limit=1000

        filtering = {
            'local_computer_id': ALL_WITH_RELATIONS,
            'system_id': ALL_WITH_RELATIONS,
            'created_at': ['gte','lte','lt','gt','eq'],
            'experiment_id': ALL_WITH_RELATIONS,
            'type': ALL_WITH_RELATIONS
        }

        serializer = PrettyJSONSerializer()


class ExperimentResource(ModelResource):
    local_computer_id = IntegerField(attribute="local_computer_id")
    name = CharField(attribute="name", null=True)

    class Meta:
        queryset = Experiment.objects.all()
        authorization = Authorization()
        authentication = Authentication()
        resource_name = 'experiment'
        filtering = {
            'local_computer_id': ALL_WITH_RELATIONS,
            'name': ALL_WITH_RELATIONS,
        }
        always_return_data = True

    serializer = PrettyJSONSerializer()
