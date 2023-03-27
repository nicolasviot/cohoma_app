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

  Translation tr (0, _index * (5 + $_context.VEHICLE_STRIP_HEIGHT))
  svg = load_from_XML ("res/svg/strip.svg")
  g << svg.strip
  monitoring aka g.strip_monitor_group
  tasks aka g.tasks_monitor_group
  
  //------ set identification data (label, icon, colors)
  toString(model.title) =:> monitoring.strip_label.text

  addChildrenTo monitoring{
   
     //icon + colors
    Component icon_strip{
      Translation _ (8, 12) 
      Scaling _ (0.3, 0.3, 0, 0)
    
      if ($type == 1){
        svg_icon = load_from_XML ("res/svg/vab.svg")
        icon << svg_icon.icon
        op_color =: icon.shape.fill.value
      } else if ($type == 2){
        Translation _ (4, 0) 
        Rotation _ (90, 0, 0)
        svg_icon = load_from_XML ("res/svg/drone.svg")
        icon << svg_icon.icon
         op_color =: icon.shape.fill.value
      } else {
        svg_icon = load_from_XML ("res/svg/robot.svg")
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
            Clock timer (100)
            Incr ellapsedIncr (0)
            TextPrinter tp
            AssignmentSequence reset_color (1){
              0 =: ellapsedIncr.state
            }
            model.data_in -> reset_color
            |-> reset_color
            
            timer.tick -> ellapsedIncr
            227 - ((ellapsedIncr.state * 100) / 4000) *  23 =:> status.fill.g
            106 + ((ellapsedIncr.state * 100) / 4000) *  100 =:> status.fill.r
            68 + ((ellapsedIncr.state * 100) / 4000) *  130 =:> status.fill.b

          }
          Component warning{
            status << svg_status.warning
          } 
        }
        model.status =:> status_switch.state
        //debug
        status_switch.warning.status.press -> model.data_in
    }

    //Capacités robots (camera/laser)
    Component robot_capacities{
      
      camera_icon aka monitoring.camera_icon
      laser_icon aka monitoring.laser_icon
      model.camera ? _context.OK_COLOR : _context.NOK_COLOR =:> camera_icon.fill.value
      model.laser ? _context.OK_COLOR : _context.NOK_COLOR =:> laser_icon.fill.value

      //DEBUG
      /*AssignmentSequence test_camera_on (1){
        1 =: model.camera
        1 =: model.laser
      }
      AssignmentSequence test_camera_off (1){
        0 =: model.camera
        0 =: model.laser
      }
      g.strip_bg.press -> test_camera_on
      g.strip_bg.release -> test_camera_off
      */
      //FIN debug
        
    }

    Component operation_mode{
      
      svg_mode = load_from_XML("res/svg/operation-mode-icons.svg")
      Translation _ (95, 12)

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

  //Gestion piege (detection dans le trap_manager)
  Component trap_detection {
    Translation _ (162, 12) 
    Scaling _ (0.6, 0.6, 0, 0)
    trap_detection_svg = load_from_XML("res/svg/icon-detect-traps.svg")
    FSM trap_detection {
      State detect_traps{
        show << trap_detection_svg.show
        1 =: model.detect_traps 
      }
      State ignore_traps{
        hide << trap_detection_svg.hide
        0 =: model.detect_traps
      }
      detect_traps -> ignore_traps (detect_traps.show.iris.press)
      detect_traps -> ignore_traps (detect_traps.show.eye.press)
      ignore_traps -> detect_traps (ignore_traps.hide.press)
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


  //locate interaction on press
  g.strip_bg.press -> model.start_locate
  g.strip_bg.release -> model.stop_locate

  //drag and drop to realocate -- -need to ask a manger to move ghost to another zone.
  Spike drop_action
  Spike drag_started
  Int off_x (0)
  Int off_y (0)

  FSM dragMachine{
    State idle {
      0 =: drag_translation.tx
      0 =: drag_translation.ty
      0=: _context.show_drop_zones_strip
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
    pressed -> idle (_context.frame_released)
    pressed -> dragging (drag_started)
    dragging -> idle (_context.frame_released)
  }
  AssignmentSequence init_dragging (1) {
    1 =: _context.show_drop_zones_strip
    model =: _context.model_of_dragged_strip
  }
  drag_started -> init_dragging
}

