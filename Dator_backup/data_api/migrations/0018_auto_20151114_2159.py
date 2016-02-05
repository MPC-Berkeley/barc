# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auth', '0006_require_contenttypes_0002'),
        ('data_api', '0017_blob_mime_type'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='localcomputer',
            name='group',
        ),
        migrations.AddField(
            model_name='localcomputer',
            name='group',
            field=models.ForeignKey(to='auth.Group', null=True),
        ),
    ]
