# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0003_auto_20150611_1955'),
    ]

    operations = [
        migrations.RenameField(
            model_name='command',
            old_name='controller',
            new_name='local_computer',
        ),
        migrations.AddField(
            model_name='command',
            name='command_type',
            field=models.IntegerField(default=0),
        ),
        migrations.AddField(
            model_name='command',
            name='is_executed',
            field=models.BooleanField(default=False),
        ),
        migrations.AddField(
            model_name='command',
            name='json_command',
            field=models.CharField(max_length=b'512', null=True),
        ),
    ]
