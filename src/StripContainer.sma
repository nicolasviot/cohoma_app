  use core
  use base
    
  import Strip

  _define_
  StripContainer (Process f, int _x, int _y) {
    
    Translation t (_x, _y) //768
    Strip strip_vab("VAB", f)
    t.tx =:> strip_vab.parent_tx
    t.ty =:> strip_vab.parent_ty
    
    Translation t2(225, 0) 
    Strip strip_agilex_1("AGILEX 1", f)
    t.tx + t2.tx =:> strip_agilex_1.parent_tx
    t.ty + t2.ty =:> strip_agilex_1.parent_ty
    
    Translation t3(225, 0) 
    Strip strip_agilex_2("AGILEX 2", f)
    t.tx + t2.tx + t3.tx =:> strip_agilex_2.parent_tx
    t.ty + t2.ty + t3.ty =:> strip_agilex_2.parent_ty

    Translation t4(225, 0) 
    Strip strip_lynx("LYNX", f)
    t.tx + t2.tx + t3.tx + t4.tx =:> strip_lynx.parent_tx
    t.ty + t2.ty + t3.ty + t4.ty =:> strip_lynx.parent_ty
    
    Translation t5(225, 0) 
    Strip strip_spot("SPOT", f)
    t.tx + t2.tx + t3.tx + t4.tx + t5.tx =:> strip_spot.parent_tx
    t.ty + t2.ty + t3.ty + t4.ty + t5.ty=:> strip_spot.parent_ty

    Translation t6(225, 0) 
    Strip strip_drone("DRONE", f)
    t.tx + t2.tx + t3.tx + t4.tx + t5.tx + t6.tx=:> strip_drone.parent_tx
    t.ty + t2.ty + t3.ty + t4.ty + t6.ty + t6.ty=:> strip_drone.parent_ty

  }