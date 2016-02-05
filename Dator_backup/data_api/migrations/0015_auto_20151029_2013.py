# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auth', '0006_require_contenttypes_0002'),
        ('data_api', '0014_auto_20150901_1640'),
    ]

    operations = [
        migrations.CreateModel(
            name='Experiment',
            fields=[
                ('id', models.AutoField(verbose_name='ID', serialize=False, auto_created=True, primary_key=True)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('updated_at', models.DateTimeField(auto_now=True)),
                ('uuid', models.CharField(max_length=128, db_index=True)),
                ('started_at', models.DateTimeField(null=True)),
                ('ended_at', models.DateTimeField(null=True)),
                ('name', models.CharField(max_length=128, db_index=True)),
                ('group', models.ManyToManyField(to='auth.Group')),
            ],
            options={
                'abstract': False,
            },
        ),
        migrations.AddField(
            model_name='blob',
            name='experiment',
            field=models.ForeignKey(to='data_api.Experiment', null=True),
        ),
        migrations.AddField(
            model_name='event',
            name='experiment',
            field=models.ForeignKey(to='data_api.Experiment', null=True),
        ),
        migrations.AddField(
            model_name='setting',
            name='experiment',
            field=models.ForeignKey(to='data_api.Experiment', null=True),
        ),
        migrations.AddField(
            model_name='signal',
            name='experiment',
            field=models.ForeignKey(to='data_api.Experiment', null=True),
        ),
    ]
