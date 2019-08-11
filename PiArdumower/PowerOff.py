#!/usr/bin/python3
# Initialisation
import time
import os
time.sleep(5)
os.system("ssh pi@192.168.1.14 'sudo shutdown -h now'")
os.system("sudo shutdown -h now")
