# Transmission-esp
## Sending Accelerometer readings from ESP to the Flask server

The data is sent according to the format {Time,nodeID,Ax,Ay,Az} and stored in a log-file
If the Wifi disconnects for whatever reason, a Last-Will message(refer MQTT Broker point 3) is sent to the broker 
If the MPU disconnects for whatever reason, the ESP sends a message to the server as "ERROR code 1".

##1. ESP32-Libraries and links:
1. Pubsubclient -> https://github.com/knolleary/pubsubclient
2. I2Cdev ->https://github.com/jrowberg/i2cdevlib
3. Processing IDE for teapot simulation -> https://processing.org/download/
4. Toxiclib for processing ide -> http://toxiclibs.org/downloads/


##2. MQTT Broker:
1. Maqiatto(online broker, easy to use during Prototype phase) -> https://www.maqiatto.com/connect
2. Setup guide for Mosquitto broker for Windows 10(just in case, better for production environment) -> http://www.steves-internet-guide.com/install-mosquitto-broker/
3. Do we require an MQTT broker? Yes :)(just in case you want to read about how MQTT works) -> http://www.steves-internet-guide.com/mqtt-works/

##3. FLASK Server:
1. Flask-MQTT -> https://flask-mqtt.readthedocs.io/en/latest/
2. Paho MQTT -> https://pypi.org/project/paho-mqtt/



