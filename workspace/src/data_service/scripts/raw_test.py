#!/home/odroid/.virtualenvs/research/bin/python

import time
from data_connection import *
from base import *
import datetime
import pytz

# create data connection
configurator = init_configurator()
print configurator.config
data_connection = DataConnection(configurator)

# signal = data_connection.get_or_create_signal("test_signal_saw_1")
signal = data_connection.get_or_create_signal("test_signal")

# print signal

# t = datetime.datetime.now(tz=pytz.UTC)
# print "got datetime {}".format(t)

# tvec = [t + datetime.timedelta(seconds=i*.01) for i in range(10)]
# tsvec = [DataConnection.utc_to_millisec(at) for at in tvec]

# # create a sawtooth wave
# signal_vector =  [[ats-tsvec[0], ats] for ats in tsvec]

# print signal_vector

# data_connection.add_signal_points(signal['id'], signal_vector)


# Uncomment here to get data
print data_connection.get_signal_points(signal['id'])
