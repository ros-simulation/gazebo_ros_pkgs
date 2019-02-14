# Holds an entity's pose and twist
string name                 # Entity's scoped name.
                            # An entity can be a model, link, collision, light, etc.
                            # Be sure to use gazebo scoped naming notation (e.g. [model_name::link_name])
geometry_msgs/Pose pose     # Pose in reference frame.
geometry_msgs/Twist twist   # Twist in reference frame.
string reference_frame      # Pose/twist are expressed relative to the frame of this entity.
                            # Leaving empty or "world" defaults to inertial world frame.

