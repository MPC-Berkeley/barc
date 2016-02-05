# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0018_auto_20151114_2159'),
    ]

    operations = [
        migrations.AddField(
            model_name='experiment',
            name='local_computer',
            field=models.ForeignKey(default=None, to='data_api.LocalComputer'),
            preserve_default=False,
        ),
    ]
