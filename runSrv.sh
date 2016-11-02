#!/bin/bash
cp tempinfo info

./server1/crazyflie-clients-python/bin/cfzmq & ./server2/crazyflie-clients-python/bin/cfzmq & ./server3/crazyflie-clients-python/bin/cfzmq & ./server4/crazyflie-clients-python/bin/cfzmq & ./server5/crazyflie-clients-python/bin/cfzmq & ./server6/crazyflie-clients-python/bin/cfzmq & ./server7/crazyflie-clients-python/bin/cfzmq & ./server8/crazyflie-clients-python/bin/cfzmq & ./server9/crazyflie-clients-python/bin/cfzmq & ./server10/crazyflie-clients-python/bin/cfzmq
