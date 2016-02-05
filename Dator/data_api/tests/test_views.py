import json
from django.contrib.auth.models import UserManager, User, Group
from django.core.urlresolvers import reverse
from django.test import TestCase, Client
from data_api.models import System, Signal, LocalComputer, Blob, Experiment, Setting
import django.utils.timezone as tmz
from data_api.views import EXPERIMENT, INCLUDE_DATA
from data_api.views import SIGNAL


class TestViews(TestCase):

    def setUp(self):
        self.client = Client()

        # when creating a System and a Local Computer
        self.system = System.objects.create(name="a_name")
        self.local_computer = LocalComputer.objects.create(name="local_computer", secret_uuid="my_uuid")

        # and a user
        um = UserManager()
        um.model = User
        self.user = um.create_user("bob", "bob@test.com", "hello")

        self.client = Client()
        self.client.login(username="bob", password="hello")

    def test_experiment_clone(self):
        # with an experiment
        group = Group.objects.create(name="a_group")
        experiment = Experiment.objects.create(name="an_experiment", local_computer=self.local_computer)
        experiment.group.add(group)

        # with settings, signals and blobs
        Signal.objects.create(experiment=experiment, local_computer=self.local_computer, name="a_signal")

        Blob.objects.create(experiment=experiment, local_computer=self.local_computer, name="a_blob",
                                     mime_type="image/jpeg")
        Setting.objects.create(experiment=experiment, local_computer=self.local_computer, key="key",
                                         value="value")

        response = self.client.post(reverse("CloneExperiment", args=(self.local_computer.id, experiment.id,)),
                                    data={'name':'new_name'})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(Experiment.objects.filter(local_computer=self.local_computer, name="new_name").count(), 1)

    def test_find_signals(self):
        # with an experiment
        group = Group.objects.create(name="a_group")
        experiment_a = Experiment.objects.create(name="experiment_a", local_computer=self.local_computer)
        experiment_a.group.add(group)
        experiment_b = Experiment.objects.create(name="experiment_b", local_computer=self.local_computer)
        experiment_b.group.add(group)

        # with signals
        Signal.objects.create(experiment=experiment_a, local_computer=self.local_computer, name="c_signal")
        Signal.objects.create(experiment=experiment_a, local_computer=self.local_computer, name="d_signal")
        Signal.objects.create(experiment=experiment_b, local_computer=self.local_computer, name="c_signal")
        Signal.objects.create(experiment=experiment_b, local_computer=self.local_computer, name="d_signal")

        # no filter
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )))
        data = json.loads(response.content)
        self.assertEqual(len(data), 4)

        # all items
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )),
                                   data={EXPERIMENT: 'experiment', SIGNAL: 'signal'})
        self.assertEqual(len(json.loads(response.content)), 4)

        # experiment_a
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )),
                                   data={EXPERIMENT: 'a'})
        self.assertEqual(len(json.loads(response.content)), 2)

        # signal_c
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )),
                                   data={SIGNAL: 'c'})
        self.assertEqual(len(json.loads(response.content)), 2)

        # signal_c and experiment_a
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )),
                                   data={EXPERIMENT: 'a', SIGNAL: 'c'})
        self.assertEqual(len(json.loads(response.content)), 1)

        # signal not z
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )),
                                   data={EXPERIMENT: 'a', SIGNAL: 'z'})
        self.assertEqual(len(json.loads(response.content)), 0)

        # experiment not z
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )),
                                   data={EXPERIMENT: 'z', SIGNAL: 'c'})
        self.assertEqual(len(json.loads(response.content)), 0)

        # experiment a or b and signal c or d
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )),
                                   data={EXPERIMENT: 'a,b', SIGNAL: 'c,d'})
        self.assertEqual(len(json.loads(response.content)), 4)

        # experiment a or z
        response = self.client.get(reverse("FindSignals", args=(self.local_computer.id, )),
                                   data={EXPERIMENT: 'a,z'})
        self.assertEqual(len(json.loads(response.content)), 2)

    def test_find_signals_with_data(self):

        experiment = Experiment.objects.create(local_computer=self.local_computer, name='an_experiment')
        signal = Signal.objects.create(local_computer=self.local_computer, name='a_signal', experiment=experiment)
        # with data
        n1 = Signal.utc_to_millisec(tmz.now())
        n2 = Signal.utc_to_millisec(tmz.now() + tmz.timedelta(seconds=1))
        signal.add_points([[1, n1], [2, n2]])

        # when requesting data
        response = self.client.get(reverse('FindSignals', args=(self.local_computer.id,)),
                                   data={INCLUDE_DATA:'true'})
        signals = json.loads(response.content)
        self.assertEqual(1, len(signals))

        # should see data
        points = signals[0]['data']
        self.assertEqual(2, len(points))
        self.assertEqual(1,  points[0][0])
        self.assertEqual(2, points[1][0])
        self.assertAlmostEqual(n1, points[0][1], 2)
        self.assertAlmostEqual(n2, points[1][1], 2)


    def test_find_signals_without_data(self):
        experiment = Experiment.objects.create(local_computer=self.local_computer, name='an_experiment')
        signal = Signal.objects.create(local_computer=self.local_computer, name='a_signal', experiment=experiment)

        # when selected data with no data
        response = self.client.get(reverse('FindSignals', args=(self.local_computer.id,)),
                                   data={INCLUDE_DATA:'true'})
        signals = json.loads(response.content)
        self.assertEqual(1, len(signals))

        # should see empty data
        points = signals[0]['data']
        self.assertEqual(0, len(points))

    def test_claim_local_computer(self):
        # with a registration_token
        self.local_computer.registration_token = "hello bob"
        self.local_computer.save()

        # should fail to claim computer with wrong token
        response = self.client.post(reverse('ClaimLocalComputer', args=(self.local_computer.id,)),
                                    data={'token': 'incorrect'})
        self.assertEquals(403, response.status_code)

        # should be able to claim computer
        response = self.client.post(reverse('ClaimLocalComputer', args=(self.local_computer.id,)),
                                    data={'token': 'hello bob'})
        self.assertEquals(200, response.status_code)
        self.assertIn(self.local_computer.group, self.user.groups.all())

    def test_get_points(self):
        signal = Signal.objects.create(system=self.system, name='a_signal')
        n1 = Signal.utc_to_millisec(tmz.now())
        n2 = Signal.utc_to_millisec(tmz.now() + tmz.timedelta(seconds=1))
        signal.add_points([[1, n1], [2, n2]])

        a = reverse('SignalData', args=(signal.id,))
        response = self.client.get(reverse('SignalData', args=(signal.id,)))
        points = json.loads(response.content)

        self.assertEqual(2, len(points))
        self.assertEqual(1,  points[0][0])
        self.assertEqual(2, points[1][0])
        self.assertAlmostEqual(n1, points[0][1], 2)
        self.assertAlmostEqual(n2, points[1][1], 2)

    def test_append_points(self):
        signal = Signal.objects.create(system=self.system, name='a_signal')
        n1 = Signal.utc_to_millisec(tmz.now())
        n2 = Signal.utc_to_millisec(tmz.now() + tmz.timedelta(seconds=1))
        a = reverse('SignalData', args=(signal.id,))
        json_data = json.dumps([[1, n1], [2, n2]])
        response = self.client.post(reverse('SignalData', args=(signal.id,)), data=json_data,
                                    content_type="application/json")
        points = signal.get_data()
        self.assertEqual(2, len(points))
        self.assertEqual(1,  points[0][0])
        self.assertEqual(2, points[1][0])
        self.assertAlmostEqual(n1, points[0][1], 2)
        self.assertAlmostEqual(n2, points[1][1], 2)

    def test_get_blob(self):
        # with a json blob
        json_blob = Blob.objects.create(local_computer=self.local_computer, name='a_signal_blob')
        # that has some data
        data = [{'a': 1, 'b': [1, 2, 3]}, {'a': 4.0, 'b': 'a_string'}]
        json_blob.set_data(json.dumps(data))

        # should be able to get the data.
        response = self.client.get(reverse('BlobData', args=(json_blob.id,)),content_type="application/octet-stream")
        json_out = json.loads(response.content)
        self.assertDictEqual(json_out[0], data[0])
        self.assertDictEqual(json_out[1], data[1])

    def test_post_blob(self):
        # with a blob
        json_blob = Blob.objects.create(local_computer=self.local_computer, name='a_signal_blob')
        # when posting data.
        data = [{'a': 1, 'b': [1, 2, 3]}, {'a': 4.0, 'b': 'a_string'}]
        response = self.client.post(reverse('BlobData', args=(json_blob.id,)), data = json.dumps(data),
                                    content_type="application/octet-stream")
        # when we refresh the data
        json_blob = Blob.objects.get(id=json_blob.id)
        persisted_data = json_blob.get_data()
        persisted_json = json.loads(persisted_data)

        # we should get the same data out.
        self.assertDictEqual(persisted_json[0], data[0])
        self.assertDictEqual(persisted_json[1], data[1])


