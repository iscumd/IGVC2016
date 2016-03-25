We have direct copyof nmea to gpsvar.

Kalman Filter will have:
-Lat
-Long
-Heading
-Speed
-Time
In addition need raw fields
-LatRaw
-LongRaw
-HeadingRaw
-SpeedRaw
-TimeRaw


Now becomes

nmea.Lat --> LatRaw
nmea.long --> LongRaw
nmea.heading --> HeadingRaw

We will user a simple Kalman Filter known as a one parameter model.
We have filtered values from some earlier data.
    Initially read from a  file etc...

Note: Filtered data includes time at which filtering was done.

Step 1:
    Given lat, long, heading, speed, time
    predict the values for t = timeraw

    Make prediciton model: speed does not change and heading does not change.
    This is a the simpilest model.
