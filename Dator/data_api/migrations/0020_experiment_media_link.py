# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0019_experiment_local_computer'),
    ]

    operations = [
        migrations.AddField(
            model_name='experiment',
            name='media_link',
            field=models.URLField(null=True, blank=True),
        ),
    ]
