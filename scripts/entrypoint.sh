#!/bin/bash

# Start detached roscore
nohup roscore &

# Start Modular Server
gunicorn --bind '0.0.0.0:5003' wsgi:app

# Wait for any process to exit
wait -n

# Exit with status of process that exited first
exit $?
