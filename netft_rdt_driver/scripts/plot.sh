#!/bin/sh

TOPIC="$1"
if [ -z "$TOPIC" ] ; then
    TOPIC="netft_data/wrench"
fi

rqt_plot \
    ${TOPIC}/force/x,${TOPIC}/force/y,${TOPIC}/force/z  \
    ${TOPIC}/torque/x,${TOPIC}/torque/y,${TOPIC}/torque/z
