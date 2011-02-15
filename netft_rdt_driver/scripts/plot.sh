#!/bin/sh

TOPIC="$1"
if [ -z "$TOPIC" ] ; then
    TOPIC="netft_data"
fi

rxplot -b 10 -p 10 \
    ${TOPIC}/linear/x,${TOPIC}/linear/y,${TOPIC}/linear/z  \
    ${TOPIC}/angular/x,${TOPIC}/angular/y,${TOPIC}/angular/z
