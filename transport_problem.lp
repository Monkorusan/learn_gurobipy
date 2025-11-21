\ Model transportation
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0x4d6b4f87e894946b
Minimize
  4 x(1,1) + 5 x(1,2) + 6 x(1,3) + 8 x(1,4) + 10 x(1,5) + 6 x(2,1)
   + 4 x(2,2) + 3 x(2,3) + 5 x(2,4) + 8 x(2,5) + 9 x(3,1) + 7 x(3,2)
   + 4 x(3,3) + 3 x(3,4) + 4 x(3,5)
Subject To
 x_11=>0: x(1,1) >= 0
 x_12=>0: x(1,2) >= 0
 x_13=>0: x(1,3) >= 0
 x_14=>0: x(1,4) >= 0
 x_15=>0: x(1,5) >= 0
 x_21=>0: x(2,1) >= 0
 x_22=>0: x(2,2) >= 0
 x_23=>0: x(2,3) >= 0
 x_24=>0: x(2,4) >= 0
 x_25=>0: x(2,5) >= 0
 x_31=>0: x(3,1) >= 0
 x_32=>0: x(3,2) >= 0
 x_33=>0: x(3,3) >= 0
 x_34=>0: x(3,4) >= 0
 x_35=>0: x(3,5) >= 0
 transportation_cost_from_factory_1__x_11_+_x_12_+_x_13_+_x_14_+_x_15_<=_500:
   x(1,1) + x(1,2) + x(1,3) + x(1,4) + x(1,5) <= 500
 transportation_cost_from_factory_2__x_21_+_x_22_+_x_23_+_x_24_+_x_25_<=_500:
   x(2,1) + x(2,2) + x(2,3) + x(2,4) + x(2,5) <= 500
 transportation_cost_from_factory_3__x_31_+_x_32_+_x_33_+_x_34_+_x_35_<=_500:
   x(3,1) + x(3,2) + x(3,3) + x(3,4) + x(3,5) <= 500
 demands_from_guest_1__x_11_+_x_21_+_x_31_<=_80: x(1,1) + x(2,1) + x(3,1)
   = 80
 demands_from_guest_2__x_12_+_x_22_+_x_32_<=_270: x(1,2) + x(2,2) + x(3,2)
   = 270
 demands_from_guest_3__x_13_+_x_23_+_x_33_<=_250: x(1,3) + x(2,3) + x(3,3)
   = 250
 demands_from_guest_4__x_14_+_x_24_+_x_34_<=_160: x(1,4) + x(2,4) + x(3,4)
   = 160
 demands_from_guest_5__x_15_+_x_25_+_x_35_<=_180: x(1,5) + x(2,5) + x(3,5)
   = 180
Bounds
End
