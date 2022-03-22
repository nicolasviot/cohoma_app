use core
use base
use gui
use animation

import Slider
import CheckBox
import gui.animation.Animator

_define_
UpperLeftMenu (Process map, Process f)
{
 svg = loadFromXML ("res/svg/icon_menu.svg")
 main_bg << svg.layer1.main_bg
 Component ui {
    Scaling sc (0, 0, 0, 0)
    Int height (0)
    Int width (0)

    /* Title 1 */
    FontFamily _ ("B612")
    FontWeight _ (75)
    FontSize _ (0, 12)
    FillColor _ (White)
    Text t (6, 17, "OSM opacity")

    /* Slider */
    Translation pos (0, 20)
    Slider s1 (f, 5, 5, 0, 100, 0)
    s1.output/100 =:> map.layers.osm.opacity

    /* Title 2 */
    Text t2 (6, 0, "Layers")
    s1.y + s1.height + 17 =:> t2.y
  
    /* Checkboxes */
    Translation pos2 (0, 0)
    FontWeight _ (50)
    TextAnchor _ (0)
    s1.y + s1.height + t2.height + 10 =:> pos2.ty
    int off_y = 0
    int nb_items = 0
    List cb_left {
      for item : map.layers {
        Component _ {
          CheckBox cb (getString (item.name), 5, off_y)
          cb.state =:> item.ctrl_visibility.state
          width aka cb.min_width
        }
        off_y += 20
        nb_items ++
      }
    }

    s1.height + nb_items * 20 + t2.height + 20 =:> height
    s1.width =:> width
  }

  Animator anim (200, 0, 1, DJN_IN_SINE, 0, 0)
  FSM menu_animation {
    State start {
      fg << clone (svg.layer1.fg)
    }
    State folded {
      fg << svg.layer1.fg
      28 =: main_bg.width, main_bg.height
      0 =: ui.sc.sx, ui.sc.sy
      2 =: main_bg.ry
    }
    State unfold {
      anim.output => ui.sc.sx, ui.sc.sy
      anim.output * (ui.s1.width - 18) + 28 =:> main_bg.width
      anim.output * (ui.s1.height * 2 - 13 + ui.pos.ty) + 28 =:> main_bg.height
    }
    State fold {
      1 - anim.output => ui.sc.sx, ui.sc.sy
      (ui.width - 18)- anim.output * (ui.width - 18) + 28 =:> main_bg.width
      (ui.height - 13 + ui.pos.ty) - anim.output * (ui.height - 13 + ui.pos.ty) + 28 =:> main_bg.height
    }
    State unfolded {
      1 =: ui.sc.sx, ui.sc.sy
      (ui.s1.width + 10) =: main_bg.width
      (ui.height + 15 + ui.pos.ty) =: main_bg.height
      5 =: main_bg.ry
    }
    start->folded (f.move) // we need this to avoid a false move event at startup
    folded->unfold (main_bg.enter, anim.start)
    unfold->unfolded (anim.end)
    unfolded->fold (map.enter, anim.start)
    fold->folded (anim.end)
  }
}