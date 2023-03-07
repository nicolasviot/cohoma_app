use core
use base
use display
use gui

import gui.widgets.IWidget

_define_
Bubble (string _label) inherits IWidget () {
  Translation offset (0, 0)

  FillColor bg_color (0,0,255)
  Rectangle bg (0,0,100,10,5,5)
  
  //TextAnchor ta(DJN_END_ANCHOR)
  FillColor fc (#ffffff)
  Text ui (0, 0, _label)
  text aka ui.text
  this.height/2 + ui.ascent/2 - 1 =:> offset.ty

  
  // bubble bg geometry according to text
  ui.x - 5 =:> bg.x
  ui.y - ui.height + 2 =:> bg.y
  ui.width + 10 =:> bg.width
  ui.height + 2 =:> bg.height


  Component date {
    Translation tr (100, 0)
    FillColor fc (#333333)
    //TextAnchor ta(DJN_END_ANCHOR)
    FontFamily _ ("B612")
    FontSize _(DJN_PX, 10)
    Text t (0, 0, "")
    WallClock wc
    //"%Hh%Mm%Ss" =: wc.format
    "%H:%M" =: wc.format
    wc.state_text =: t.text
    //this.width - t.width =:> tr.tx 
  }

  // whole geometry
  ui.width =: this.min_width
  ui.height =: this.min_height
  bg.width + 100 + date.t.width =: this.preferred_width
  bg.height =: this.preferred_height
  // ui.width =: this.min_width
  // this.preferred_width == -1 ? ui.width : this.preferred_width =: this.preferred_width
  // ui.height =: this.min_height
  // this.preferred_height == -1 ? ui.height : this.preferred_height =: this.preferred_height

}
