# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0010_auto_20150626_2219'),
    ]

    operations = [
        migrations.CreateModel(
            name='Setting',
            fields=[
                ('id', models.AutoField(verbose_name='ID', serialize=False, auto_created=True, primary_key=True)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('updated_at', models.DateTimeField(auto_now=True)),
                ('uuid', models.CharField(max_length=128, db_index=True)),
                ('key', models.CharField(max_length=128, db_index=True)),
                ('value', models.CharField(max_length=128)),
                ('local_computer', models.ForeignKey(to='data_api.LocalComputer', null=True)),
                ('system', models.ForeignKey(to='data_api.System', null=True)),
            ],
            options={
                'abstract': False,
            },
        ),
        migrations.AlterField(
            model_name='signal',
            name='name',
            field=models.CharField(max_length=128, db_index=True),
        ),
    ]
