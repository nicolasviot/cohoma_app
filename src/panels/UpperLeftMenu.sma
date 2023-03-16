use core
use base
use gui
use animation

import widgets.Slider
import widgets.CheckBox
import gui.animation.Animator

_define_
UpperLeftMenu (Process map, Process _context, Process _model_manager, Process f)
{
  //context aka _context

  Translation _ ($_context.LEFT_PANEL_WIDTH, $_context.TOP_BAR_HEIGHT)

  svg = load_from_XML ("res/svg/icon_menu.svg")
  main_bg << svg.layer1.main_bg

  Component ui {
    Scaling sc (0, 0, 0, 0)
    Int height (0)
    Int width (0)

    /* Title 1 */
    FontFamily _ ("B612")
    FontWeight _ (DJN_BOLD)
    FontSize _ (0, 12)
    FillColor _ (White)
    Text t (6, 17, "Opacity")

    /* Slider */
    Translation pos (0, 20)
    Slider s1 (f, 5, 5, 0, 100, 0)
    //s1.output/100 =:> map.layers.osm.opacity
    s1.output / 100 =:> _context.veil_opacity

    /* Title 2 */
    Text t2 (6, 0, "Layers")
    s1.y + s1.height + 17 =:> t2.y
  
    /* Checkboxes */
    Translation pos2 (0, 0)
    FontWeight _ (DJN_NORMAL)
    TextAnchor _ (0)
    s1.y + s1.height + t2.height + 10 =:> pos2.ty
    int off_y = 0

    List check_box_list {
      for model : _model_manager.layers {
        Component _ {
          CheckBox cb (toString (model.name), 5, off_y)

          // Model --> View
          model.is_visible =:> cb.is_checked

          // View --> Model
          cb.unchecked -> {
            0 =?: model.is_visible
          }
          cb.checked -> {
            1 =?: model.is_visible
          }
        }
        off_y += 20
      }
    }

    s1.height + check_box_list.size * 20 + t2.height + 20 =:> height
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
      (ui.width - 18) - anim.output * (ui.width - 18) + 28 =:> main_bg.width
      (ui.height - 13 + ui.pos.ty) - anim.output * (ui.height - 13 + ui.pos.ty) + 28 =:> main_bg.height
    }
    State unfolded {
      1 =: ui.sc.sx, ui.sc.sy
      ui.s1.width + 10 =: main_bg.width
      ui.height + 15 + ui.pos.ty =: main_bg.height
      5 =: main_bg.ry
    }
    start -> folded (f.move) // we need this to avoid a false move event at startup
    folded -> unfold (main_bg.enter, anim.start)
    unfold -> unfolded (anim.end)
    unfolded -> fold (map.enter, anim.start)
    fold -> folded (anim.end)
  }
}