You are an expert research scientist with a PhD in computer science, focused on MEMS sensors, linear algebra, time series analysis, frequency analysis, kalman filters, STM X-cube-mems libraries, ... 

We want to create an algorithm that can:

- track orientation of azimuth and altitude
- ignore drift (we do not care about drift)
- track abrubt changes in the orientation
- continue working for long periods of time (hours on end - even weeks)
- continue working after restart of device

So, what we want is to place a mast of 1 meter outside with a box on top. In this box, we have a PCB with the ISM330DHCX (@https://www.st.com/resource/en/datasheet/ism330dhcx.pdf ) in WDS (West = x, Down = y, South = Z) position. 

We will only focus on making the algorithm "library". We will not care about getting data from ISM, we can assume everything we need is handed to us.

The mast will be rather static and not moving, however, since it is placed outside, there will be wind, cars passing it by, ... so there will be quite some noise, vibrations, ... that we have to ignore. If the device slowly moves due to wind, we do not care. Meaning, we have a threshold for azimuth and a threshold for altitude. If the angle change is lower than this threshold within a given time interval (2 seconds or something), then we do not care. We only care about abrupt changes that exceed the threshold.

If needed, we can have a calibration phase where we can assume the device is not moving or vibrating and we can do a calibration procedure.
We can also assume that our accelerometer readings are always slightly tilted, making it easier to perform azimuth tracking, since our values A = [Ax, Ay, Az] will never have Ax = Ay = 0. There will always be some effect of gravity (do research here - @https://www.sciencedirect.com/science/article/pii/S0167610523001666 ).

So, our goal: when we do an installation (and after possible calibration), we start monitoring. If there is drift or slow movement, we ignore this. However, as soon as someone rotates or tilts the device over the threshold in a fast manner and stays here for at least some seconds (or something big hits the sensor and knocks it over), we go into a validation state. We stay in this state for `n` minutes (can range from 10 minutes to 4 hours, this is something we can set/change depending on environment). If we are still knocked over/moved after this time period, we will raise an event and go into this event state.

There might be some things we are missing, but it is up to you to find these gaps.

We need to get some research done first. Use jupyter notebooks to create some tests for yourself so that you can do good research. Not only that, try to really find what the best way is for us to tackle this problem. Once you have a good understanding, create a LaTex paper using the adonis (@https://github.com/NicholasMamo/adonis-template )  template. This paper should read as a to-the-point research paper with solutions.

It is VERY important to note that we work using the KISS principle and fail-first, fail-fast, improve-fast. Meaning we want to do everything as simple as possible. Do not overcomplicate things, do not try to make nice structures/classes/.... just get results fast is the idea here.