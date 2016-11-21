# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0023_mediadata'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='mediadata',
            name='experiment',
        ),
        migrations.RemoveField(
            model_name='mediadata',
            name='group',
        ),
        migrations.RemoveField(
            model_name='experiment',
            name='media_link',
        ),
        migrations.DeleteModel(
            name='MediaData',
        ),
    ]
