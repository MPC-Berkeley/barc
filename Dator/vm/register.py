import json
import os
import requests
import uuid
import sys

SERVER = 'server'
REGISTRATION_TOKEN = 'registration_token'
NAME = 'name'
UUID = 'secret_uuid'
ID = 'id'

class Register():
    def __init__(self):
        self.data = {
            's': [SERVER, "http://dator.forge9.com"],
            'r': [REGISTRATION_TOKEN, "a secret key"],
            'u': [UUID, str(uuid.uuid4())],
            'n': [NAME, "your computer name"]
        }

    def display_and_select(self):
        for key in self.data.keys():
            print "[{}]: {}".format(self.data[key][0], self.data[key][1])
        print "Enter choice to edit field or <enter> to accept values"
        print "> ",

        selection = sys.stdin.readline().strip()

        if selection:
            selection_char = selection[0]
            if selection_char in self.data.keys():
                print "Enter new value for {} : [{}]".format(self.data[selection_char][0], self.data[selection_char][1])
                value = sys.stdin.readline().strip()
                if len(value) > 0:
                    self.data[selection_char][1] = value
            else:
                print "Selection not recognized"
                print "hit <enter> to continue."
                sys.stdin.readline()

            return True
        else:
            return False

    def register(self):
        config_dict = {}
        for key in self.data.keys():
            config_dict[self.data[key][0]] = self.data[key][1]
        url = config_dict[SERVER] + '/api/v1/local_computer/?format=json'
        print "Registering at " + url
        response = requests.post(url, data=json.dumps(config_dict),
                                 headers={'content_type':"application/json"})
        new_config = json.loads(response.content)
        print "Received config with id {}".format(new_config['id'])
        filename = "config.json"
        print "Output file [{}]".format(filename)
        print "Enter new filename or <enter> to save> "
        new_name = sys.stdin.readline()
        if len(new_name.strip()) > 0:
            filename = new_name
        with open(filename, 'w') as output:
            output.write(json.dumps(new_config, indent=4))

    @classmethod
    def clear_screen(cls):
        # clear and reset cursor only works for linux
        sys.stdout.write("\x1b[2J\x1b[H")

if __name__ == "__main__":
    register = Register()
    register.clear_screen()
    while register.display_and_select():
        register.clear_screen()
    register.register()
