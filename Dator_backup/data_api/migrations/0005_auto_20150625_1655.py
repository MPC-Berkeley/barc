# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0004_auto_20150624_0108'),
    ]

    operations = [
        migrations.AddField(
            model_name='localcomputer',
            name='command_refresh_sec',
            field=models.IntegerField(default=10),
        ),
        migrations.AddField(
            model_name='localcomputer',
            name='secret_uuid',
            field=models.CharField(default='dummy', max_length=128),
            preserve_default=False,
        ),
    ]
