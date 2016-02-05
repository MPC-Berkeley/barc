# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0006_auto_20150626_0032'),
    ]

    operations = [
        migrations.AlterField(
            model_name='command',
            name='is_executed',
            field=models.BooleanField(default=False, db_index=True),
        ),
        migrations.AlterField(
            model_name='command',
            name='type',
            field=models.IntegerField(default=0, db_index=True),
        ),
    ]
