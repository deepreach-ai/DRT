#!/bin/bash
# Generate self-signed certificate for HTTPS/WebXR support
echo "🔐 Generating self-signed SSL certificate..."
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes -subj '/CN=TeleopServer'
echo "✅ Generated key.pem and cert.pem"
echo "   Use these with: python run_server.py --ssl-key key.pem --ssl-cert cert.pem"
