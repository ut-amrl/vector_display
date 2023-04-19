#!/bin/bash
cd maps
git pull --all
git add . && git commit -m "modify maps for spot-tour-guide UGs"
git push -u origin sadanand/tour-guide
git pull --all

cd ..
git pull --all
git add . && git commit -m "modify maps for spot-tour-guide UGs"
git push -u origin sadanand/tour-guide
git pull --all

cd ../amrl_maps
git pull --all
git add . && git commit -m "modify maps for spot-tour-guide UGs"
git push -u origin sadanand/tour-guide
git pull --all

