\ Model transportation
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0x66398e7cdd56a423
Minimize
  4 x(1,1) + 5 x(2,1) + 6 x(3,1) + 8 x(4,1) + 10 x(5,1) + 6 x(1,2)
   + 4 x(2,2) + 3 x(3,2) + 5 x(4,2) + 8 x(5,2) + 9 x(1,3) + 7 x(2,3)
   + 4 x(3,3) + 3 x(4,3) + 4 x(5,3)
Subject To
 transportation_cost_from_factory_1__x_11_+_x_21_+_x_31_+_x_41_+_x_51_<=_500:
   x(1,1) + x(2,1) + x(3,1) + x(4,1) + x(5,1) <= 500
 transportation_cost_from_factory_2__x_12_+_x_22_+_x_32_+_x_42_+_x_52_<=_500:
   x(1,2) + x(2,2) + x(3,2) + x(4,2) + x(5,2) <= 500
 transportation_cost_from_factory_3__x_13_+_x_23_+_x_33_+_x_43_+_x_53_<=_500:
   x(1,3) + x(2,3) + x(3,3) + x(4,3) + x(5,3) <= 500
 demands_from_guest_1__x_11_+_x_12_+_x_13_<=_80: x(1,1) + x(1,2) + x(1,3)
   = 80
 demands_from_guest_2__x_21_+_x_22_+_x_23_<=_270: x(2,1) + x(2,2) + x(2,3)
   = 270
 demands_from_guest_3__x_31_+_x_32_+_x_33_<=_250: x(3,1) + x(3,2) + x(3,3)
   = 250
 demands_from_guest_4__x_41_+_x_42_+_x_43_<=_160: x(4,1) + x(4,2) + x(4,3)
   = 160
 demands_from_guest_5__x_51_+_x_52_+_x_53_<=_180: x(5,1) + x(5,2) + x(5,3)
   = 180
Bounds
End
