use core
use base
use gui

_define_
Slider (Process f, int _x, int _y, int _lower, int _upper)
{
  Translation pos (_x, _y)
  x aka pos.tx
  y aka pos.ty
  Double width (0)
  Double height (0)
  BoundedValue bv (_lower, _upper, _upper)
  output aka bv.result

  svg = loadFromXML ("res/svg/slider.svg")

  main_bg << svg.bg
  bg << svg.bg_line
  fg << svg.fg_line
  main_bg.width =: width
  main_bg.height =: height
  fg.width = $bg.width

  Translation button_pos ($bg.width, 0)
  Switch halo (idle) {
  	Component idle
  	Component moving {
  	  halo << svg.halo
  	}
  }

  BoundedValue pos_bv (0, $bg.width, $bg.width)
  Int i_output ($bg.width)
  output =:> i_output
  button << svg.button
  prop << svg.property
  FSM button_fsm {
  	State idle {
  		#ffffff =: button.fill.value
      "" =: prop.text
  	}
  	State hover {
  		#ffa700 =: button.fill.value
      toString (i_output) + " %" =:> prop.text
  	}
  	State moving {
  		Double offset_x (0)
      Double init_tx (0)
      Double delta_x (0)
      #ffa700 =: button.fill.value
      button_pos.tx =: init_tx
      button.press.x =: offset_x
      f.move.x - offset_x => delta_x
      delta_x + init_tx => pos_bv.input
      pos_bv.result => button_pos.tx, fg.width
      pos_bv.result*((_upper - _lower)/$bg.width) + _lower =:> bv.input
  	  toString (i_output) + " %" =:> prop.text
  	}
  	idle->hover (button.enter)
  	hover->idle (button.leave)
  	{hover,idle}->moving (button.press)
  	idle->hover (button.move)
  	moving->idle (button.release)
  }
  button_fsm.state =:> halo.state
}