# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0016_localcomputer_user'),
    ]

    operations = [
        migrations.AddField(
            model_name='blob',
            name='mime_type',
            field=models.CharField(max_length=128, null=True, db_index=True),
        ),
    ]
