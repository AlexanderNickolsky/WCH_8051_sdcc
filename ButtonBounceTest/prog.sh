#!/bin/sh
FILE="`ls *.cbp`"
./isp55e0 -f bin/Release/${FILE%%.*}.bin