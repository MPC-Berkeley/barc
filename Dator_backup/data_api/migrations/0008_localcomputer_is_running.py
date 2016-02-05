# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0007_auto_20150626_0520'),
    ]

    operations = [
        migrations.AddField(
            model_name='localcomputer',
            name='is_running',
            field=models.BooleanField(default=False),
        ),
    ]
