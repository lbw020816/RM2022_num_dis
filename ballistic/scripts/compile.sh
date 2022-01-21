#！/bin/bash
cd `dirname $0`
mv exp_com.py exp.py
easycython exp.py
rm -rf build
rm exp.html
rm exp.c
mv exp.*.so exp.so
mv exp.py exp_com.py
