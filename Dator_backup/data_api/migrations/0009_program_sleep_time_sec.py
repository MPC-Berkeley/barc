# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0008_localcomputer_is_running'),
    ]

    operations = [
        migrations.AddField(
            model_name='program',
            name='sleep_time_sec',
            field=models.IntegerField(default=1),
        ),
    ]
