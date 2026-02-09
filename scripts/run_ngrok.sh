#!/bin/bash

# Define local path
LOCAL_NGROK="./bin/ngrok"
NGROK_CMD="ngrok"

# Check for local binary first
if [ -f "$LOCAL_NGROK" ]; then
    NGROK_CMD="$LOCAL_NGROK"
    echo "Using local ngrok binary at $LOCAL_NGROK"
elif ! command -v ngrok &> /dev/null; then
    echo "ngrok could not be found."
    echo "Please install it from https://ngrok.com/download"
    echo "or run: brew install ngrok/ngrok/ngrok"
    echo "Alternatively, download the zip manually and place 'ngrok' in ./bin/"
    exit 1
fi

echo "Starting ngrok on port 8000..."
echo "Copy the HTTPS URL below and use it as the Server URL in the VR client."
echo ""

# Check for local config
if [ -f "./ngrok.yml" ]; then
    echo "Using local configuration file: ./ngrok.yml"
    $NGROK_CMD http 8000 --config ./ngrok.yml
else
    $NGROK_CMD http 8000
fi
