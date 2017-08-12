#!/bin/bash
FILENAME=$1
echo $FILENAME
python src/parser.py $FILENAME;
swipl -l out/$FILENAME.pl -f src/planner.pl -g "test,nl." -t halt