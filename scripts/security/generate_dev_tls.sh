#!/usr/bin/env bash
set -e
mkdir -p certs
openssl req -x509 -nodes -newkey rsa:4096 -days 397 -keyout certs/server.key -out certs/server.crt -subj "/CN=localhost" -addext "subjectAltName=DNS:localhost"
chmod 600 certs/server.key certs/server.crt
echo "certs/server.key and certs/server.crt generated"
