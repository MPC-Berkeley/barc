import json
import uuid
import requests
import datetime as tmz
import pytz
import delorean

class DataConnection(object):

    def _get_csrf(self):
        """
        :return: Get the csrf from the local session
        """
        response = self.client.get(url="{}/noop/".format(self.configurator.get_config()['server']))
        return response.cookies['csrftoken']

    def __init__(self, configurator):
        self.configurator = configurator
        self.client = requests.session()
        self.csrf = self._get_csrf()

    def _api_url(self, resource_name):
        """
        Get restful endpoint for resource type
        :param resource_name: The type of resource requested (e.g. "local_computer")
        :return: The base api url to connect to the data server
        """
        return "{}/api/v1/{}/?format=json".format(self.configurator.get_config()['server'], resource_name)

    def _item_url(self, resource_name, resource_id):
        """
        Get restful endpoint for a specific instance of a resource.
        :param resource_name: The type of resource requested
        :param id: The id of the resource
        :return: A specific resource instance
        """
        return "{}/api/v1/{}/{}/?format=json".format(self.configurator.get_config()['server'], resource_name, resource_id)

    def _data_item_url(self, resource_name, resource_id):
        """
        Add data points to the given resource
        :param resource_name: The type of resource requested
        :param id: The id of the resource
        :return: A specific data_api resource instance
        """
        return "{}/data_api/v1/{}/{}/?format=json".format(self.configurator.get_config()['server'], resource_name, resource_id)

    def register(self, registration_token=None, file_name=None):
        """
        Call to register a new local computer.
        """
        config = self.configurator.get_config()
        if registration_token:
            config["registration_token"] = registration_token
        config["secret_uuid"] = str(uuid.uuid4())
        response = requests.post(self._api_url('local_computer'), data=json.dumps(self.configurator.get_config()),
                                 headers=self.post_header())
        new_config = json.loads(response.content)
        self.configurator.set_config(new_config)

    def update_config(self, config_location):
        """
        :param config_location: Location of local config file
        Update local config with global config.
        """
        config = self.configurator.get_config()
        url = self._item_url('local_computer', config['id'])
        response = requests.get(url, headers= self.sec_header())
        if self.check_response_ok(response):
            updated_config = json.loads(response.content)
            for key in updated_config.keys():
                config[key] = updated_config[key]
            self.configurator.write_config(config_location)

    def set_local_computer_status(self, is_running):
        """
        :param isRunning:
        """
        config = self.configurator.get_config()
        config['is_running'] = is_running
        url = url = self._item_url('local_computer', config['id'])
        response = requests.put(url, data=json.dumps(self.configurator.get_config()), headers=self.sec_header())

    def get_new_commands(self):
        """
        :return: Commands from the server for this local computer that should be executed.
        """
        config = self.configurator.get_config()
        id = config["id"]
        url = self._api_url('command') + "&is_local_computer_id={}&is_executed=false".format(id)
        response = requests.get(url, headers = self.sec_header())
        if self.check_response_ok(response):
            return json.loads(response.content)['objects']
        return []

    def deactivate_command(self, command):
        """
        Indicate that a command has been executed.
        :param command:
        :return:
        """
        config = self.configurator.get_config()
        id = config["id"]
        command['is_executed'] = True
        url = self._item_url('command', command['id']) + "&is_local_computer_id={}".format(id)
        response = requests.put(url, data=json.dumps(command), headers=self.sec_header())
        self.check_response_ok(response)

    def get_program(self, program_id):
        """
        :param program_id:
        :return: the program object or None if it doesn't exist
        """
        config = self.configurator.get_config()
        url = self._item_url('program', program_id)
        response = self.client.get(url, headers=self.sec_header())
        if self.check_response_ok(response):
            return json.loads(response.content)
        return None

    def add_signal_points(self, signal_id, signal_points):
        """ Add sorted time/value points to a signal
        :param signal_id: The id of the signal to add points to
        :param signal_points: [[<t1>,<v1>],[<t2>,<v2>
        """
        config = self.configurator.get_config()
        url = self._data_item_url('signal', signal_id)
        response = self.client.post(url, data=json.dumps(signal_points), headers=self.post_header())
        if not DataConnection.check_response_ok(response):
            print "Error posting signal data {}".format(response.content)

    def add_signal_points_by_name(self, signal_name, signal_points):
        """
        Add time/value points to a signal object belonging to this local_computer.
        :param signal_name:
        :param signal_points: points in format[[<value>,<millisec>], ...]
        """
        config = self.configurator.get_config()

        url = self._api_url('signal')
        response = self.client.get(url, params={'name': signal_name, 'local_computer_id': config['id']},
                                headers=self.sec_header())
        signal_id = json.loads(response.content)['objects'][0]['id']
        self.add_signal_points(signal_id, signal_points)

    def get_signal_points(self, signal_id):
        """
        Get the data points from the given signal.
        :param signal_id:
        :return: a list of signal point in format[[<value>,<millisec>], ...]
        """
        config = self.configurator.get_config()

        url = self._data_item_url('signal', signal_id)
        response = self.client.get(url,headers=self.sec_header())
        return json.loads(response.content)

    def get_signal_points_by_name(self, name):
        """
        Get the indicated signal from the local computer
        :param name: The name of the signal
        :return: a list of time/value points [[<value1>,<millisec>] ...]
        """
        config = self.configurator.get_config()
        url = self._api_url('signal')
        response = self.client.get(url, params={'name': name, 'local_computer_id': config['id']},
                                headers=self.sec_header())
        return self.get_signal_points(json.loads(response.content)['objects'][0]['id'])

    def get_or_create_signal(self, signal_name):
        """
        Get or create a signal object for this local computer
        :param signal_name:
        :return: The signal object dict.
        """
        config =self.configurator.get_config()
        url = self._api_url('signal')
        params = {'name': signal_name, 'local_computer_id': config['id']}

        response = self.client.get(url, params=params, headers=self.sec_header())
        if len(json.loads(response.content)['objects']) == 0:
            self.client.post(url, data=json.dumps(params), headers=self.post_header())
            response = self.client.get(url, params=params, headers=self.sec_header())

        return json.loads(response.content)['objects'][0]

    def get_or_create_setting(self, key):
        """
        Get or create a setting object for this local computer
        :param setting: Setting key
        :return: The setting object dict.
        """
        config =self.configurator.get_config()
        url = self._api_url('setting')
        params = {'key': key, 'local_computer_id': config['id']}
        response = self.client.get(url, params=params, headers=self.sec_header())
        if len(json.loads(response.content)['objects']) == 0:
            self.client.post(url, data=json.dumps(params), headers=self.post_header())
            response = self.client.get(url, params=params, headers=self.sec_header())

        return json.loads(response.content)['objects'][0]

    def write_setting(self, key, value):
        config =self.configurator.get_config()
        url = self._api_url('setting')
        params = {'key': key, 'local_computer_id': config['id']}
        response = self.client.get(url, params=params, headers=self.sec_header())

        setting = json.loads(response.content)['objects'][0]
        url = self._item_url("setting", setting['id'])
        params['value'] = value
        self.client.put(url, data=json.dumps(params), headers=self.post_header())

    def get_or_create_blob(self, blob_name, content_type="application/json"):
        """
        Get or create a new blob object for this local computer
        :param blob_name:
        :return: the blob object dict.
        """
        config =self.configurator.get_config()
        url = self._api_url('blob')
        params = {'name': blob_name, 'local_computer_id': config['id']}
        response = self.client.get(url, params=params, headers=self.sec_header())
        if len(json.loads(response.content)['objects']) == 0:
            params['mime_type']=content_type
            response = self.client.post(url, data=json.dumps(params), headers=self.post_header())
            params.pop('mime_type')
            response = self.client.get(url, params=params, headers=self.sec_header())

        return json.loads(response.content)['objects'][0]

    def set_blob_data(self, id, blob_data):
        """
        Store data in a blob object.
        :param id:
        :param blob_data: String representation of blob to store
        :return:
        """
        url = self._data_item_url('blob', id)
        headers = self.post_header()
        headers['content-type'] = 'application/octet_stream'
        response = self.client.post(url, data=blob_data, headers=headers)
        if not DataConnection.check_response_ok(response):
            print "Error posting blob data {}".format(response.content)

    def set_blob_data_by_name(self, name, blob_data):
        config = self.configurator.get_config()
        url = self._api_url('blob')
        response = self.client.get(url, params={'name': name, 'local_computer_id': config['id']},
                                headers=self.sec_header())
        self.set_blob_data(json.loads(response.content)['objects'][0]['id'], blob_data)


    def get_blob_data(self, id):
        url = self._data_item_url('blob', id)
        response = self.client.get(url, headers=self.sec_header())
        return response.content

    def get_blob_data_by_name(self, name):
        config = self.configurator.get_config()
        url = self._api_url('blob')
        response = self.client.get(url, params={'name': name, 'local_computer_id': config['id']},
                                headers=self.sec_header())
        return self.get_blob_data(json.loads(response.content)['objects'][0]['id'])

    def create_event(self, event_type, info):
        config = self.configurator.get_config()
        url = self._api_url('event')
        response = self.client.post(url, data=json.dumps({'type': event_type, 'info': info, 'local_computer_id': config['id']}),
                                    headers=self.post_header())
        if not self.check_response_ok(response):
            print "Error creating event {}: {}".format(event_type, info)

    def sec_header(self, base_header=None):
        auth_header = {'auth_key': self.configurator.get_config()["secret_uuid"], 'content-type': 'application/json'}
        if base_header is None:
            return auth_header
        auth_header.update(base_header)

    def post_header(self):
        auth_header = {'auth_key': self.configurator.get_config()["secret_uuid"], 'content-type': 'application/json', "X-CSRFToken": self.csrf}
        return auth_header



    @classmethod
    def check_response_ok(cls, response):
        """
        :return: True if http response is a 200 class response.  False info otherwise.
        """
        if 200 <= response.status_code < 300:
            return True
        else:
            response_string = "WARNING Command lookup failed: status: {} reason: {}".format(response.status_code, response.reason)
            if response.content:
                response_string = response_string + "\n" + response.content
            if response.reason:
                response_string = response_string + "\n" + response.reason
            print response_string
            return False





    @classmethod
    def millisec_to_utc(cls, millisec):
        return tmz.datetime.fromtimestamp(float(millisec), tz=pytz.UTC)

    @classmethod
    def utc_to_millisec(cls, dt):
        return delorean.Delorean(dt, timezone="UTC").epoch()
