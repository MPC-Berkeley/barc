from uuid import uuid4
from django.db import models
from django.contrib.auth.models import Group, User
from django.db.models.signals import pre_save, post_save
from django.dispatch import receiver
from data_api import file_provider
import pandas as pd
import django.utils.timezone as tmz
import pytz

import delorean
import dator
from django.conf import settings

class SystemModel(models.Model):
    created_at = models.DateTimeField(auto_now_add=True, null=True)
    updated_at = models.DateTimeField(auto_now=True, null=True)
    uuid = models.CharField(max_length=128, db_index=True)

    class Meta:
        abstract = True

class Event(SystemModel):
    """
    An event is used to record controller specific events for correlation with data signals.
    """
    group = models.ManyToManyField(Group)
    type = models.CharField(max_length=32)
    info = models.TextField(null=True)
    local_computer = models.ForeignKey('LocalComputer', null=True)
    system = models.ForeignKey('System', null=True)
    experiment = models.ForeignKey('Experiment', null=True)

    def __unicode__(self):
        return "{}:{}".format(self.local_computer_id, self.type)

class System(SystemModel):
    """
    A system is a group of related LocalComputers that  coordinate actions and signals with each other.
    """
    group = models.ManyToManyField(Group)
    name = models.CharField(max_length=128)
    timezone = models.CharField(max_length=32)
    shifts = models.ManyToManyField('Shift')

    def __unicode__(self):
        return self.name

class Shift(SystemModel):
    """
    A Shift is used to record the beginning and the end of an experiment
    """
    name = models.CharField(max_length=128)
    ended_at = models.DateTimeField(null=True)

class LocalComputer(SystemModel):
    """
    A LocalComputer system is a cpu capable of loading a program, recording data from sensors and operating actuators.
    """
    group = models.ForeignKey(Group, null=True)
    user = models.ForeignKey(User, on_delete=models.SET_NULL, null=True)
    name = models.CharField(max_length=128)
    registration_token = models.CharField(max_length=128)
    secret_uuid = models.CharField(max_length=128)
    system = models.ForeignKey('System', null=True)
    command_refresh_sec = models.IntegerField(default=10)
    is_running = models.BooleanField(default=False)

    def __unicode__(self):
        return self.name

# No-op
COMMAND_NOOP = 0
# shut down local_computer listener
COMMAND_DONE = 1
# load and start the indicated program on the local computer
COMMAND_LOAD_PROGRAM = 2
# stop the indicated program on the local computer
COMMAND_STOP_PROGRAM = 3


class Command(SystemModel):
    """
    Commands are enumerated json messages for LocalComputers.
    When command has been successfully executed, the is_executed flag is set to True
    """
    local_computer = models.ForeignKey('LocalComputer')
    type = models.IntegerField(default=COMMAND_NOOP, db_index=True)
    json_command = models.CharField(max_length="512", null=True)
    is_executed = models.BooleanField(default=False, db_index=True)

    def __unicode__(self):
        return "{}:{}:{}".format(self.local_computer_id, self.type, self.created_at)

class Program(SystemModel):
    """
    A loadable script/code file that can be run on a local computer.
    A program will be run periodically with with a pause of the indicated
    sleep_time between sucessive runs.
    """
    group = models.ManyToManyField(Group)
    code = models.TextField(null=True)
    description = models.TextField(null=True)
    name = models.CharField(max_length=128)
    sleep_time_sec = models.FloatField(default=1.0)

    def __unicode__(self):
        return self.name

class Map(SystemModel):
    """
    A map is a list of known signals with semantic meaning.
    """
    group = models.ManyToManyField(Group)
    name = models.CharField(max_length=128)
    controller = models.ForeignKey('LocalComputer')

    def __unicode__(self):
        return self.name


ACTUATOR = 1
SENSOR = 2


class MapPoint(SystemModel):

    map = models.ForeignKey('Map')
    point_type = models.IntegerField(default=SENSOR)
    name = models.CharField(max_length=128)
    path = models.CharField(max_length=128)
    controller = models.ForeignKey('LocalComputer')

    def __unicode__(self):
        return self.name


class Signal(SystemModel):
    """
    A time signal of floats.
    """
    group = models.ManyToManyField(Group)
    name = models.CharField(max_length=128, db_index=True)
    system = models.ForeignKey('System', null=True)
    local_computer = models.ForeignKey('LocalComputer', null=True)
    experiment = models.ForeignKey('Experiment', null=True)

    def __unicode__(self):
        return self.name

    def add_points(self, frames):
        """Add points to the signal
        :param frames [[<float value>,<float value2>, <time in millisec>],...]
        empty values must be formatted as nan
        """
        settings.SIGNAL_PROVIDER.startup()
        string_frames = []
        for frame in frames:
            string_frames.append('['+ ','.join(["{:.15}".format(float(datum)) for datum in frame]) + ']')

        settings.SIGNAL_PROVIDER.append_data(self.uuid,
                                    ''.join([string_frame for string_frame in string_frames]))


    def get_data(self):
        settings.SIGNAL_PROVIDER.startup()
        try:
            data = settings.SIGNAL_PROVIDER.get_blob(self.uuid)

            tokens = data.split("]")
            points = []
            for token in tokens:
                if token != '':
                    ts = token[1:].split(",")
                    points+=[[float(t) for t in ts]]
            return points
        except:
            return []

    @classmethod
    def millisec_to_utc(cls, millisec):
        return tmz.datetime.fromtimestamp(float(millisec), tz=pytz.UTC)

    @classmethod
    def utc_to_millisec(cls, dt):
        epoch = delorean.Delorean(dt, timezone="UTC").epoch
        if not hasattr(epoch, '__call__'):
            return delorean.Delorean(dt, timezone="UTC").epoch
        else:
            return delorean.Delorean(dt, timezone="UTC").epoch()

    def get_time_series(self):
        values, dates = self.get_data()
        return pd.TimeSeries(values, index=dates)

    def clear(self):
        settings.SIGNAL_PROVIDER.startup()
        settings.SIGNAL_PROVIDER.clear(self.uuid)


class Setting(SystemModel):
    group = models.ManyToManyField(Group)
    key = models.CharField(max_length=128, db_index=True)
    value = models.CharField(max_length=128)
    local_computer = models.ForeignKey('LocalComputer', null=True)
    system = models.ForeignKey('System', null=True)
    experiment = models.ForeignKey('Experiment', null=True)

    def __unicode__(self):
        return '{},{}'.format(self.key, self.value)


class Blob(SystemModel):
    group = models.ManyToManyField(Group)
    name = models.CharField(max_length=128, db_index=True)
    system = models.ForeignKey('System', null=True)
    local_computer = models.ForeignKey('LocalComputer', null=True)
    experiment = models.ForeignKey('Experiment', null=True)
    mime_type = models.CharField(max_length=128, null=True, db_index=True)

    def __unicode__(self):
        return self.name

    def get_data(self):
        settings.BLOB_PROVIDER.startup()
        data = settings.BLOB_PROVIDER.get_blob(self.uuid)
        return data

    def set_data(self, json_data):
        settings.BLOB_PROVIDER.startup()
        settings.BLOB_PROVIDER.write_blob(self.uuid, json_data)


class Experiment(SystemModel):
    group = models.ManyToManyField(Group)
    started_at = models.DateTimeField(null=True)
    ended_at = models.DateTimeField(null=True)
    name = models.CharField(max_length=128, db_index=True)
    local_computer = models.ForeignKey(LocalComputer)

    def __unicode__(self):
        return u"{}-{}-{}".format(self.name, self.started_at, self.ended_at)

    def clone(self, name):
        experiment = Experiment.objects.create(local_computer=self.local_computer,
                                               name=name)
        groups = self.group.all()
        experiment.group.add(*groups)
        for signal in self.signal_set.all():
            sig = Signal.objects.create(local_computer=self.local_computer,
                                  experiment=experiment,
                                  name=signal.name,
                                  system=signal.system)
            sig.group.add(*groups)
        for setting in self.setting_set.all():
            set = Setting.objects.create(local_computer=self.local_computer,
                                  experiment=experiment,
                                  key=setting.key,
                                  value=setting.value)
            set.group.add(*groups)
        for blob in self.blob_set.all():
            blb = Blob.objects.create(local_computer=self.local_computer,
                                  experiment=experiment,
                                  name=blob.name,
                                  system=blob.system,
                                  mime_type=blob.mime_type)
            blb.group.add(*groups)
        return experiment


class LocalSignalTag(models.Model):
    signal = models.ForeignKey('Signal')
    uploaded = models.BooleanField(default=False)



@receiver(pre_save, sender=Command)
@receiver(pre_save, sender=LocalComputer)
@receiver(pre_save, sender=Map)
@receiver(pre_save, sender=MapPoint)
@receiver(pre_save, sender=Program)
@receiver(pre_save, sender=Shift)
@receiver(pre_save, sender=Signal)
@receiver(pre_save, sender=System)
@receiver(pre_save, sender=Setting)
@receiver(pre_save, sender=Event)
@receiver(pre_save, sender=Blob)
@receiver(pre_save, sender=Experiment)
def set_uuid(sender, instance, **kwargs):
    """
    Register all SystemModel derived classes to set uuid
    """
    if not instance.uuid:
        instance.uuid = str(uuid4())

@receiver(pre_save, sender=LocalComputer)
def add_group(sender, instance, **kwargs):
    if instance.group is None:
        time_stamp = str(Signal.utc_to_millisec(tmz.now()))[0:20]
        if len(instance.name) > 60:
            name = instance.name[0:60] + time_stamp
        else:
            name = instance.name + time_stamp
        group = Group.objects.create(name = name)
        instance.group = group
    if instance.user is None:
        time_stamp = str(Signal.utc_to_millisec(tmz.now()))[0:20]
        if len(instance.name) > 60:
            name = instance.name[0:60] + time_stamp
        else:
            name = instance.name + time_stamp
        user = User.objects.create(username = name)
        user.groups.add(instance.group)
        instance.user = user
