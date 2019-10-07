#!/bin/bash

sudo apt-get update
sudo apt-get upgrade

# Installation de python3 et pip3
sudo apt-get install python3 python3-pip

# Installation du paquet ros pour python
python3 -m pip install rospkg --user

# Installation de la bibliothèque pyserial pour les données via USB
python3 -m pip install pyserial --user

# Installation de la bibliothèque irsensors pour lire les données des capteurs IR
python3 -m pip install irsensors --user
