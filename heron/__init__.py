#!/usr/bin/env python3
# coding: utf-8


try:
    from serial import Serial
except ImportError:
    from os import system
    print("Please, accept to install:\n" + \
        "\t python3-pip and pyton librairies \"pyserial\" and \"rospkg\"\n\n" + \
        "in order to use ower ROS package.")
    system("sudo apt install python3-pip")
    system("python3 -m pip install pyserial rospkg --user")

from heron.hello import coucou
from heron.battery import Battery


__all__ = [
    "coucou"
    "Battery"
]


if __name__ == "__main__":
    try:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        pass
