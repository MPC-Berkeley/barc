#!/usr/bin/env python

# Simple script to try to automatically configure wifi as the default route
# for the odroid
# author: Kiet Lam <kiet.lam@berkeley.edu>

import subprocess
import re
import requests


def main():
    try:
        _ = requests.get('http://www.google.com', timeout=5)
        print 'Wifi is up and ready!'
        return True
    except Exception as err:
        # Try to configure wifi here
        p1 = subprocess.Popen(['ip', 'route', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p1.communicate()
        pattern = re.compile(r'(?P<ip>.*)\.[0-9]+\/[0-9]*\s+dev\s+(?P<interface>wlan[0-9]+).*proto\s+kernel')
        result = re.search(pattern, out)
        if result:
            wifi_ip = result.groupdict()['ip'] + '.1'
            wifi_interface = result.groupdict()['interface']
            print 'wifi_ip:', wifi_ip
            print 'wifi_interface:', wifi_interface

            p2 = subprocess.Popen(['ip', 'route', 'change', 'default', 'dev', wifi_interface, 'via', wifi_ip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out, err = p2.communicate()


if __name__ == '__main__':
    main()
