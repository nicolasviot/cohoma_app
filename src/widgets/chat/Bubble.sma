use core
use base
use display
use gui

import gui.widgets.IWidget

_define_
Bubble (string _label) inherits IWidget () {
  Translation offset (0, 0)
  
  FillColor _ (0,0,255)
  Rectangle bg (0,0,100,10,5,5)
  
  //TextAnchor ta(DJN_END_ANCHOR)
  FillColor fc (#ffffff)
  Text ui (0, 0, _label)
  text aka ui.text
  this.height/2 + ui.ascent/2 - 1 =:> offset.ty

  ui.width =: this.min_width
  this.preferred_width == -1 ? ui.width : this.preferred_width =: this.preferred_width
  ui.height =: this.min_height
  this.preferred_height == -1 ? ui.height : this.preferred_height =: this.preferred_height

  ui.x =:> bg.x
  ui.y - ui.height =:> bg.y
  ui.width =:> bg.width
  ui.height =:> bg.height
}
