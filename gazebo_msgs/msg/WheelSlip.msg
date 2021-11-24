# This is a message that holds data to describe the wheel slip for a set of wheels.
#
# The wheel slip is defined by
#  * the lateral slip of the wheel, as an angle from the direction of travel (radians), and
#  * the longitudinal slip of the wheel, as a percentage of the current wheel's speed
#
# Each wheel is uniquely identified by its name.
#
# This message consists of multiple arrays.
# All arrays in this message should be the same size.
# This is the only way to uniquely associate the wheel name with the correct slip values.

string[] name
float64[] lateral_slip
float64[] longitudinal_slip