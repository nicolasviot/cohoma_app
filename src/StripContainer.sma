  use core
  use base
    
  import Strip

  _define_
  StripContainer (Process f, int _x, int _y) {
    Translation t (_x, _y) //768
    Strip strip1("uav 1", f)
    t.tx =:> strip1.parent_tx
    t.ty =:> strip1.parent_ty
    Translation t2(400, 0) 
    Strip strip2("uav 2", f)
    t.tx + t2.tx =:> strip2.parent_tx
    t.ty + t2.ty =:> strip2.parent_ty

  }