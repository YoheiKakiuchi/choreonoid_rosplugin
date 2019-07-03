#!/bin/bash

pkill -9 choreonoid
##trap "pkill choreonoid -g 0" SIGINT SIGKILL SIGTERM

echo $@ >&2

choreonoid $@
