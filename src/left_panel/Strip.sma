use core
use gui
use display

_define_
Strip (Process _context, Process _model, int _index)
{
  model aka _model
  op_color aka model.operator_color.value
  type aka model.type

  Translation drag_translation (0,0)

  Translation tr (5, _index * (5 + $_context.VEHICLE_STRIP_HEIGHT))
  svg = load_from_XML ("res/svg/strip.svg")
  g << svg.strip
  monitoring aka g.strip_monitor_group
  tasks aka g.tasks_monitor_group
  traps aka g.trap_group

  
  //------ set identification data (label, icon, colors)
  toString(model.title) =:> monitoring.strip_label.text

  addChildrenTo monitoring{
   
     //icon + colors
     //todo changer en IF car statique
    Component icon_strip{
      Translation _ (20, 30) 
      Scaling _ (0.5, 0.5, 0, 0)
      Rotation _ (90, 0, 0)

      if ($type == 1){
        svg_icon = load_from_XML ("res/svg/robot.svg")
        icon << svg_icon.icon
        op_color =: icon.shape.fill.value
      } else if ($type == 2){
        svg_icon = load_from_XML ("res/svg/drone.svg")
        icon << svg_icon.icon
         op_color =: icon.shape.fill.value
      } else {
        svg_icon = load_from_XML ("res/svg/vab.svg")
        icon << svg_icon.icon
         op_color =: icon.shape.fill.value
      }
    }

    //health-status
    Component life_status{
        svg_status = load_from_XML("res/svg/status-icons.svg")
        Translation _ (70, 12)
        Switch status_switch (error){
          Component error{
            status << svg_status.error
          }
          Component ok{
            status << svg_status.ok
          }
          Component warning{
            status << svg_status.warning
          } 
        }
        model.status =:> status_switch.state
    }

    Component operation_mode{
      
      svg_mode = load_from_XML("res/svg/operation-mode-icons.svg")
      Translation _ (70, 32)

      Switch mode_switch (Unknown){
          Component Unknown{
            mode_icon << clone(svg_mode.wait) //used twice...
          }
          Component Manual{
            mode_icon << clone(svg_mode.wait)
          }
          Component TeleOP{
            mode_icon << svg_mode.manual
          } 
          Component Auto{
            mode_icon << svg_mode.auto
          }
        }
     
      model.operation_mode_status =:> mode_switch.state
    }

  }

  // --- Tasks et planification

  Spike show_new_task_panel
  Spike hide_new_task_panel

  FSM new_task_panel{
    State idle{

    }
    State visible{
      Rectangle bg (0,0, 100, 100, 0,0)
      Rectangle close (10,10, 10, 10, 0,0)
      close.press -> hide_new_task_panel
    }
    idle -> visible (show_new_task_panel)
    visible -> idle (hide_new_task_panel)
  }
  tasks.btn_add_task.button_show_bg.press -> show_new_task_panel


  // ---- traps detection


  //locate interaction on press
  g.strip_bg.press -> model.start_locate
  g.strip_bg.release -> model.stop_locate

  Spike drop_action
  Spike drag_started

   Int off_x (0)
   Int off_y (0)
  
  //drag and drop to realocate -- -need to ask a manger to move ghost to another zone.
  FSM dragMachine{
    State idle {
      0 =: drag_translation.tx
      0 =: drag_translation.ty
    }
    State pressed{
      g.strip_bg.press.x =: off_x
      g.strip_bg.press.y =: off_y
      Double dist (0)
      (g.strip_bg.move.x - off_x) * (g.strip_bg.move.x - off_x) + (g.strip_bg.move.y - off_y) * (g.strip_bg.move.y - off_y) =:> dist
      dist > 40 -> drag_started
    }
    State dragging{
      |-> model.stop_locate
      g.strip_bg.move.x - off_x =:> drag_translation.tx
      g.strip_bg.move.y - off_y =:> drag_translation.ty
    }
    
    idle -> pressed (g.strip_bg.press)
    pressed -> idle (g.strip_bg.release)
    pressed -> dragging (drag_started)
    dragging -> idle (g.strip_bg.release)
  }
}

