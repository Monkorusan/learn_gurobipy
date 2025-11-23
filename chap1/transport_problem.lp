\ Model transportation
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0x66398e7cdd55db9d
Minimize
  4 x(1,1) + 5 x(2,1) + 6 x(3,1) + 8 x(4,1) + 10 x(5,1) + 6 x(1,2)
   + 4 x(2,2) + 3 x(3,2) + 5 x(4,2) + 8 x(5,2) + 9 x(1,3) + 7 x(2,3)
   + 4 x(3,3) + 3 x(4,3) + 4 x(5,3)
Subject To
 transportation_cost_from_factory_1___x_11+x_21+x_31+x_41+x_51<=500: x(1,1)
   + x(2,1) + x(3,1) + x(4,1) + x(5,1) <= 500
 transportation_cost_from_factory_2___x_12+x_22+x_32+x_42+x_52<=500: x(1,2)
   + x(2,2) + x(3,2) + x(4,2) + x(5,2) <= 500
 transportation_cost_from_factory_3___x_13+x_23+x_33+x_43+x_53<=500: x(1,3)
   + x(2,3) + x(3,3) + x(4,3) + x(5,3) <= 500
 demands_from_guest_1___x_11+x_12+x_13<=80: x(1,1) + x(1,2) + x(1,3) = 80
 demands_from_guest_2___x_21+x_22+x_23<=270: x(2,1) + x(2,2) + x(2,3)
   = 270
 demands_from_guest_3___x_31+x_32+x_33<=250: x(3,1) + x(3,2) + x(3,3)
   = 250
 demands_from_guest_4___x_41+x_42+x_43<=160: x(4,1) + x(4,2) + x(4,3)
   = 160
 demands_from_guest_5___x_51+x_52+x_53<=180: x(5,1) + x(5,2) + x(5,3)
   = 180
Bounds
End
