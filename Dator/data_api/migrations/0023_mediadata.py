# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auth', '0006_require_contenttypes_0002'),
        ('data_api', '0022_auto_20160406_0722'),
    ]

    operations = [
        migrations.CreateModel(
            name='MediaData',
            fields=[
                ('id', models.AutoField(verbose_name='ID', serialize=False, auto_created=True, primary_key=True)),
                ('created_at', models.DateTimeField(auto_now_add=True, null=True)),
                ('updated_at', models.DateTimeField(auto_now=True, null=True)),
                ('uuid', models.CharField(max_length=128, db_index=True)),
                ('uploaded', models.BooleanField(default=False)),
                ('local_location', models.FilePathField()),
                ('experiment', models.ForeignKey(to='data_api.Experiment', null=True)),
                ('group', models.ManyToManyField(to='auth.Group')),
            ],
            options={
                'abstract': False,
            },
        ),
    ]
