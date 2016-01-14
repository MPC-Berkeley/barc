import json
from django.test import LiveServerTestCase
from data_api.models import Command, LocalComputer, COMMAND_NOOP, Signal, System, Blob, Event, Setting
from vm.base import Configurator
from vm.data_connection import DataConnection
import datetime
import pytz


class TestDataConnection(LiveServerTestCase):
    """
    Data Connection API Tests
    """
    def setUp(self):
        # With a configuration pointed to localhost
        self.local_computer = LocalComputer.objects.create(name="a_computer")

        self.configurator = Configurator()
        config = self.configurator.get_config()
        config['id'] = self.local_computer.id
        config['server'] = self.live_server_url

        self.data_connection = DataConnection(self.configurator)

    def tearDown(self):
        pass

    def test_get_new_commands(self):
        # with a new command
        json_command = json.dumps({'a':1, 'b':'c'})
        Command.objects.create(type=COMMAND_NOOP, local_computer=self.local_computer, json_command=json_command)

        # should get command
        commands = self.data_connection.get_new_commands()
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0]['type'], COMMAND_NOOP)
        self.assertEqual(commands[0]['json_command'], json_command)

    def test_add_signal_points(self):
        # with a signal and persisted data
        signal = Signal.objects.create(name='a_signal')
        n1 = Signal.utc_to_millisec(datetime.datetime.now(tz=pytz.UTC))
        n2 = Signal.utc_to_millisec(datetime.datetime.now(tz=pytz.UTC) + datetime.timedelta(seconds=1))
        json_data = [[1, n1], [2, n2]]
        self.data_connection.add_signal_points(signal.id, json_data)

        # should find persisted data
        points = signal.get_data()
        self.assertEqual(2, len(points))
        self.assertEqual(1,  points[0][0])
        self.assertEqual(2, points[1][0])
        self.assertAlmostEqual(n1, points[0][1], 2)
        self.assertAlmostEqual(n2, points[1][1], 2)

        # should download points by id
        downloaded_points = self.data_connection.get_signal_points(signal.id)
        self.assertEqual(2, len(downloaded_points))
        self.assertEqual(1,  downloaded_points[0][0])
        self.assertEqual(2, downloaded_points[1][0])
        self.assertAlmostEqual(n1, downloaded_points[0][1], 2)
        self.assertAlmostEqual(n2, downloaded_points[1][1], 2)

    def test_add_blob_data(self):
        blob = Blob.objects.create(name="a_blob")
        some_data = "ASDF ASDF ASDR e3airijqofwajOIEGHO34UOIUT290TJALKJDSJF"
        self.data_connection.set_blob_data(blob.id, some_data)

        new_data = self.data_connection.get_blob_data(blob.id)

        self.assertEquals(some_data, new_data)


    def test_add_signal_points_by_name(self):
        # with a signal
        signal = Signal.objects.create(name='a_signal', local_computer=self.local_computer)
        n1 = Signal.utc_to_millisec(datetime.datetime.now(tz=pytz.UTC))
        n1 = Signal.utc_to_millisec(datetime.datetime.now(tz=pytz.UTC))
        n2 = Signal.utc_to_millisec(datetime.datetime.now(tz=pytz.UTC) + datetime.timedelta(seconds=1))
        json_data = [[1, n1], [2, n2]]
        self.data_connection.add_signal_points_by_name(signal.name, json_data)

        # should find persisted data
        points = signal.get_data()
        self.assertEqual(2, len(points))
        self.assertEqual(1,  points[0][0])
        self.assertEqual(2, points[1][0])
        self.assertAlmostEqual(n1, points[0][1], 2)
        self.assertAlmostEqual(n2, points[1][1], 2)

        # should download points by name
        downloaded_points = self.data_connection.get_signal_points_by_name(signal.name)
        self.assertEqual(2, len(downloaded_points))
        self.assertEqual(1,  downloaded_points[0][0])
        self.assertEqual(2, downloaded_points[1][0])
        self.assertAlmostEqual(n1, downloaded_points[0][1], 2)
        self.assertAlmostEqual(n2, downloaded_points[1][1], 2)

    def test_add_blob(self):
        # with a blob and uploaded data
        blob = Blob.objects.create(name='another blob', local_computer=self.local_computer)
        data = range(10,20)
        blob_data = json.dumps(data)
        self.data_connection.set_blob_data(blob.id, blob_data)

        # server should persist the data
        self.assertIsNotNone(blob.get_data())
        self.assertSequenceEqual(json.loads(blob.get_data()), data)

        # should download data
        download_data = json.loads(self.data_connection.get_blob_data(blob.id))
        self.assertSequenceEqual(download_data, data)

    def test_add_blob_by_name(self):
        # with a blob and uploaded data
        blob = Blob.objects.create(name='another blob', local_computer=self.local_computer)
        data = range(10,20)
        blob_data = json.dumps(data)
        self.data_connection.set_blob_data_by_name(blob.name, blob_data)

        # server should persist the data
        self.assertIsNotNone(blob.get_data())
        self.assertSequenceEqual(json.loads(blob.get_data()), data)

        # should download data
        download_data = json.loads(self.data_connection.get_blob_data_by_name(blob.name))
        self.assertSequenceEqual(download_data, data)

    def test_get_or_create_signal(self):
        # when creating a signal
        json_signal = self.data_connection.get_or_create_signal("a signal")
        # and looking it up in the db.
        created_signal = Signal.objects.get(name="a signal")
        # should create a signal
        self.assertIsNotNone(created_signal)
        # with proper id
        self.assertEqual(json_signal['id'], created_signal.id)

    def test_get_or_create_blob(self):
        self.data_connection.get_or_create_blob("a blob")
        self.assertIsNotNone(Blob.objects.get(name="a blob"))

    def test_get_or_create_setting(self):
        gain = self.data_connection.get_or_create_setting("gain")
        self.assertIsNotNone(Setting.objects.get(key="gain", local_computer_id=self.local_computer.id))
        self.assertEqual(gain['value'], "")

    def test_set_setting(self):
        Setting.objects.create(key="gain", local_computer_id=self.local_computer.id, value=1.0)
        gain = self.data_connection.get_or_create_setting("gain")
        self.assertEqual(float(gain['value']), 1.0)

        self.data_connection.write_setting("gain", 2.0)
        self.assertEqual(Setting.objects.get(key="gain", local_computer_id=self.local_computer.id).value, "2.0")
        gain = self.data_connection.get_or_create_setting("gain")
        self.assertEqual(gain['value'], "2.0")

    def test_create_event(self):
        self.data_connection.create_event("a type", "some text")
        self.assertIsNotNone(Event.objects.get(type="a type", info="some text", local_computer=self.local_computer))

    def test_create_multiple_signals(self):
        signal1 = self.data_connection.get_or_create_signal("signal1")
        self.assertEqual(signal1['id'], Signal.objects.get(name="signal1", local_computer=self.local_computer).id)
        signal2 = self.data_connection.get_or_create_signal("signal2")
        self.assertEqual(signal2['id'], Signal.objects.get(name="signal2", local_computer=self.local_computer).id)