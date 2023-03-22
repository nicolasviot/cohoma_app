use core
use gui
use display

_define_
Strip (Process _context, Process _model, int _index)
{
  model aka _model
  op_color aka model.operator_color.value
  type aka model.type

  Translation tr (5, _index * (5 + $_context.VEHICLE_STRIP_HEIGHT))
  svg = load_from_XML ("res/svg/strip.svg")
  g << svg.strip
  monitoring aka g.strip_monitor_group
  tasks aka g.tasks_monitor_group
  traps aka g.trap_group

  
  //------ set identification data (label, icon, colors)

  //label
  toString(model.title) =:> monitoring.strip_label.text

  addChildrenTo monitoring{
   
     //icon + colors
     //todo changer en IF car statique
    Component icon_strip{
      Translation _ (20, 30) 
      Scaling _ (0.5, 0.5, 0, 0)
      Rotation _ (90, 0, 0)

      SwitchList iconSwitchList (1){
        Component _ {
          svg_icon = load_from_XML ("res/svg/vab.svg")
          icon << svg_icon.icon
          op_color =: icon.shape.fill.value
        }
        Component _ {
          svg_icon = load_from_XML ("res/svg/robot.svg")
          icon << svg_icon.icon
          op_color =: icon.shape.fill.value
        
        }
        Component _ {
          svg_icon = load_from_XML ("res/svg/drone.svg")
          icon << svg_icon.icon
          op_color =: icon.shape.fill.value
          
        }
      }
      type =: iconSwitchList.index
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



  //locate interaction on press
  g.strip_bg.press -> model.start_locate
  g.strip_bg.release -> model.stop_locate
}

