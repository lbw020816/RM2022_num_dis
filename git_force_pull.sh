#!/bin/bash
cd `dirname $0`
git fetch --all
git reset --hard origin/master
