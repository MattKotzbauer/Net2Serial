
TCP/IP to UART converter compatile with virtual environment or Arduino Mega 2560.

Files:

base_converter.py: base converter with no security additions
c_converter.c: C implementation of converter - does not yet have security additions
secure_converter.py: implementation of converter with full security. SSL/TLS keys must be generated on localhost - can do via SSL with the following command: 

openssl req -x509 -newkey rsa:4096 -keyout key_file.key -out cert.crt -days 365 -nodes



