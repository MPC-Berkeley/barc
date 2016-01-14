# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data_api', '0001_initial'),
    ]

    operations = [
        migrations.AddField(
            model_name='localcomputer',
            name='registration_token',
            field=models.CharField(default='', max_length=128),
            preserve_default=False,
        ),
        migrations.AlterField(
            model_name='localcomputer',
            name='group',
            field=models.ManyToManyField(to='auth.Group', null=True),
        ),
        migrations.AlterField(
            model_name='localcomputer',
            name='system',
            field=models.ForeignKey(to='data_api.System', null=True),
        ),
    ]
