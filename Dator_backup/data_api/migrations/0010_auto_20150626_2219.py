# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0009_program_sleep_time_sec'),
    ]

    operations = [
        migrations.AddField(
            model_name='signal',
            name='local_computer',
            field=models.ForeignKey(to='data_api.LocalComputer', null=True),
        ),
        migrations.AddField(
            model_name='signal',
            name='system',
            field=models.ForeignKey(to='data_api.System', null=True),
        ),
    ]
