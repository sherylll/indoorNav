# indoorNav
For the iNETS lab

Useful links:
filter acceleration to cancel out the influence of gravity
https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-raw-data

compute orientation (with geomagnetic sensor, see src code from Android):
https://developer.android.com/guide/topics/sensors/sensors_position.html


# Problems:
v & d grow due to accel. offset.
time offset (might millis->micros)?

# Run Server:
Go to /server, run python manage.py runserver, open browser and visit localhost:8000/map/


# Quaternion approach:

https://github.com/arduino-libraries/MadgwickAHRS.git

https://github.com/kriswiner/LSM9DS1/blob/master/quaternionFilters.ino
https://github.com/kriswiner/LSM9DS1/blob/master/LSM9DS1_MS5611_BasicAHRS_t3.ino
