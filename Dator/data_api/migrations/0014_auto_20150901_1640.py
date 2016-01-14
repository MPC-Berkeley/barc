# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auth', '0006_require_contenttypes_0002'),
        ('data_api', '0013_event'),
    ]

    operations = [
        migrations.CreateModel(
            name='Blob',
            fields=[
                ('id', models.AutoField(verbose_name='ID', serialize=False, auto_created=True, primary_key=True)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('updated_at', models.DateTimeField(auto_now=True)),
                ('uuid', models.CharField(max_length=128, db_index=True)),
                ('name', models.CharField(max_length=128, db_index=True)),
                ('group', models.ManyToManyField(to='auth.Group')),
                ('local_computer', models.ForeignKey(to='data_api.LocalComputer', null=True)),
                ('system', models.ForeignKey(to='data_api.System', null=True)),
            ],
            options={
                'abstract': False,
            },
        ),
        migrations.RemoveField(
            model_name='binaryblob',
            name='group',
        ),
        migrations.RemoveField(
            model_name='signalblob',
            name='signal',
        ),
        migrations.AddField(
            model_name='shift',
            name='ended_at',
            field=models.DateTimeField(null=True),
        ),
        migrations.DeleteModel(
            name='BinaryBlob',
        ),
        migrations.DeleteModel(
            name='SignalBlob',
        ),
    ]
