#!/bin/bash
FILENAME=$1
echo $FILENAME
python src/parser.py $FILENAME;
swipl out/$FILENAME.pl src/planner.pl