# Testing different between rate sleep and timer callbacks

## Concept

Publishers publish on 2 topics, one at slow rate 1 Hz and one at higher rate of 20 Hz
with the simulated clock almost following system clock, one expects call periods for the slow message to be 1 s and fast message 50 ms

## Run options with a simulated clock through 'myclock' node

### Testing rate sleep with different simulated clock update rates

ros2 launch test_timers start.launch.py test_rates:=true update_rate:=10.0
ros2 launch test_timers start.launch.py test_rates:=true update_rate:=100.0
ros2 launch test_timers start.launch.py test_rates:=true update_rate:=1000.0

### Testing timer callbacks with different simulated clock update rates

ros2 launch test_timers start.launch.py test_timers:=true update_rate:=10.0
ros2 launch test_timers start.launch.py test_timers:=true update_rate:=100.0
ros2 launch test_timers start.launch.py test_timers:=true update_rate:=1000.0

## Run options with a gazebo as the source of simulated clock

### Testing rate sleep with different gazebo simulated clock update_rates

* ros2 launch test_timers start.launch.py test_rates::=true use_gazebo:=true
* ros2 launch test_timers start.launch.py test_rates:=true use_gazebo:=true gazebo_param_file:=gazebo_100HzUpdateRate_params.yaml

### Testing timer callbacks with different simulated clock update rates

* ros2 launch test_timers start.launch.py test_timers:=true use_gazebo:=true
* ros2 launch test_timers start.launch.py test_timers:=true use_gazebo:=true gazebo_param_file:=gazebo_100HzUpdateRate_params.yaml

## Results

### Success at update rate = 100 Hz

[test_timers-2] [INFO] [1691413014.957349480] [timer_based.timers_test]: Publishing: 'FAST message 31 call period 0.049487 s, expected 0.050 s'
[test_timers-2] [INFO] [1691413015.007518151] [timer_based.timers_test]: Publishing: 'SLOOOOW message 32 call period 1.000013 s, expected 1.0 s'

### Failure at update rate = 10 Hz

[test_timers-2] [INFO] [1691412946.954410982] [timer_based.timers_test]: Publishing: 'FAST message 17 call period 0.100915 s, expected 0.050 s'
[test_timers-2] [INFO] [1691412947.053357592] [timer_based.timers_test]: Publishing: 'SLOOOOW message 18 call period 0.999568 s, expected 1.0 s
