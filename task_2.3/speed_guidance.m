function desired_speed = speed_guidance(U_max, distance,R_LOS,k_attenuate)

desired_speed = U_max; %-sigmoid(distance,R_LOS)*k_attenuate;