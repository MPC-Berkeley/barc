# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0005_auto_20150625_1655'),
    ]

    operations = [
        migrations.RenameField(
            model_name='command',
            old_name='command_type',
            new_name='type',
        ),
    ]
