import json
import os
from uuid import uuid4
import multiprocessing
import requests
import time
import sys
from data_connection import DataConnection


class Configurator(object):
    """
    Manages a config for the Local Computer
    """

    def __init__(self, **kwargs):
        if "filename" in kwargs:
            with open(kwargs["filename"], 'r') as input:
                self.config = json.loads(input.read())
        else:
            self.config = {
                'server': 'http://localhost:8000',
                'secret_uuid': '123412341234',
                'registration_token': 'abcd',
                'name': "a_local_computer"
            }

    def write_config(self, filename):
        """
        Save current config to a json file.
        """
        with open(filename, 'w') as output:
            output.write(json.dumps(self.config, indent=4))

    def get_config(self):
        return self.config

    def set_config(self, config):
        self.config = config


CONFIG_LOCATION = "default.cfg"


def init_configurator(config_location=None):
    """
    Register local computer if not done previously.
    :return: The configurator for this local computer
    """

    CONFIG_LOCATION=config_location
    if os.path.isfile(CONFIG_LOCATION):
        configurator = Configurator(filename=CONFIG_LOCATION)
        print "Found local configurator at {}".format(CONFIG_LOCATION)
    else:
        configurator = Configurator()
        configurator.write_config(CONFIG_LOCATION)

        data_connection = DataConnection(configurator)
        my_reg_token = str(uuid4())
        print "Registering to {} with token {}".format(configurator.get_config()['server'], my_reg_token)
        data_connection.register(my_reg_token, CONFIG_LOCATION)
        configurator.write_config(CONFIG_LOCATION)

    return configurator


def periodic_eval(refresh_time_sec, program, should_stop, shared_val, data_connection):
    while not should_stop.value:
        try:
            eval(compile(program, '<string>', 'exec'))
            time.sleep(refresh_time_sec)
        except BaseException as e:
            print ("Error running uploaded program {}".format(e))
            if e.message:
                print e.message
    return periodic_eval


class WorkerPool(object):

    def __init__(self, data_connection):
        self.job_list = {}
        self.shared_val = multiprocessing.Value('i',0)
        self.data_connection = data_connection

    def start_program(self, program_id, refresh_time_sec, program):
        if program_id not in self.job_list.keys():
            should_stop = multiprocessing.Value('b', False)
            self.job_list[program_id] = [should_stop,
                                         multiprocessing.Process(target=periodic_eval, args=(
                                             refresh_time_sec,
                                             program,
                                             should_stop,
                                             self.shared_val,
                                             self.data_connection
                                         ))]
            self.job_list[program_id][1].start()
        else:
            print "Program id {} already running".format(program_id)

    def stop_program(self, program_id):
        try:
            job = self.job_list[program_id]
            self.job_list[program_id][0].value = True
            self.job_list[program_id][1].join(20)
            del self.job_list[program_id]
            print "Stopped job for program id {}".format(program_id)
        except BaseException as e:
            print "Failed to stop program {}".format(program_id)

    def stop(self):
        for program_id in self.job_list.keys():
            self.stop_program(program_id)

COMMAND_NOOP = 0
COMMAND_DONE = 1
COMMAND_LOAD_PROGRAM = 2
COMMAND_STOP_PROGRAM = 3


class CommandHandler(object):

    def __init__(self, worker_pool, data_connection):
        self.worker_pool = worker_pool
        self.data_connection = data_connection
        self.handler_map = {
            COMMAND_LOAD_PROGRAM: self.handle_load,
            COMMAND_STOP_PROGRAM: self.handle_stop,
        }

    def handle_commands(self, commands):
        done = False
        for command in commands:

            print ("Rcx command {}".format(command["type"]))
            if command['type'] == COMMAND_NOOP:
                pass
            elif command['type'] == COMMAND_DONE:
                done = True
            else:
                self.handler_map[command['type']](command)

            data_connection.deactivate_command(command)
        return done

    def handle_load(self, command):
        program_command = json.loads(command['json_command'])
        program = self.data_connection.get_program(program_command['program_id'])
        print ("Starting program {}".format(program['id']))
        self.worker_pool.start_program(program['id'], program['sleep_time_sec'], program['code'])

    def handle_stop(self, command):
        program_command = json.loads(command['json_command'])
        program = self.data_connection.get_program(program_command['program_id'])

        self.worker_pool.stop_program(program['id'])



if __name__ == '__main__':
    """
    Main loop.  Handle commands until done.
    """
    if len(sys.argv) != 2:
        print "usage: python base <config file>"
        sys.exit()

    configurator = init_configurator(sys.argv[1])
    data_connection = DataConnection(configurator)
    data_connection.update_config(CONFIG_LOCATION)
    worker_pool = WorkerPool(data_connection)
    command_handler = CommandHandler(worker_pool, data_connection)

    done = False
    data_connection.set_local_computer_status(is_running=True)
    print "Connected to " + configurator.get_config()['server']
    print "Polling for commands.  <ctrl-c> to exit."
    while not done:
        try:
            commands = data_connection.get_new_commands()
            done = command_handler.handle_commands(commands)
            time.sleep(configurator.get_config()['command_refresh_sec'])
        except KeyboardInterrupt:
            print "Stopping"
            done = True

    worker_pool.stop()
    data_connection.set_local_computer_status(is_running=False)
    print("got shared_value {}".format(worker_pool.shared_val.value))

    print("Received done command.  Shutting down.")




