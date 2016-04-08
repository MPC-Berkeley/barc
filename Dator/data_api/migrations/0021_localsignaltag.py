# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0020_experiment_media_link'),
    ]

    operations = [
        migrations.CreateModel(
            name='LocalSignalTag',
            fields=[
                ('id', models.AutoField(verbose_name='ID', serialize=False, auto_created=True, primary_key=True)),
                ('uploaded', models.BooleanField(default=False)),
                ('signal', models.ForeignKey(to='data_api.Signal')),
            ],
        ),
    ]
