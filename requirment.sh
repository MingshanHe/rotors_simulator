#!/bin/bash
sudo python3 -m pip install python-uinput -i https://pypi.tuna.tsinghua.edu.cn
sudo python3 -m pip install pygame -i https://pypi.tuna.tsinghua.edu.cn
sudo ln -s /usr/bin/python3 /usr/bin/python
sudo ln -s libboost_python38.so libboost_python3.so
sudo apt-get install python3.8-dev