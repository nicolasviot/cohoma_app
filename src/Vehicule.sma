use core
use gui
use base
use animation

import gui.animation.Animator
import behavior.NotDraggableItem

_native_code_
%{
    #include "cpp/coords-utils.h"
%}


_define_
Vehicule (Process map, Process _context, Process _model, Process _svg)
{
    //APP-6A: Symbologie militaire interarmées de l'OTAN

    //context aka _context
    model aka _model
 
    Translation screen_translation (0, 0)

    Rotation rot (0, 0, 0)
    model.heading_rot =:> rot.a

    // [insert beautiful graphics here]
    icon << clone (_svg.icon)
    model.color =: icon.shape.fill.value

    /*Switch switch_type  (vab) {
        Component vab {
        }
        Component agilex1 {
        }
        Component agilex2 {
        }
        Component lynx {
        }
        Component spot {
        }
        Component drone {
        }
    }
    model.type =:> switch_type.state*/

    // Update the position via "screen_translation" in function of lat/lon and current zoom level
    NotDraggableItem not_draggable_item (map, model.lat, model.lon, screen_translation.tx, screen_translation.ty)


    ///// HIGHLIGHT ANIMATION ON REQUEST /////

    //TODO compute radius according to the map center
    Double radius(60)
    Double distance_to_center_pix (100)

    ClampMin clamp_radius (60, 60)
    distance_to_center_pix =:> clamp_radius.input
    clamp_radius.result =:> radius

    Animator radius_anim (1000, 60, 5, DJN_IN_OUT_SINE, 1, 0)
    distance_to_center_pix =:> radius_anim.min

    20 =: radius_anim.fps
    radius_anim.output =:> radius

    FSM locate_FSM {
        State idle

        State animate {
            OutlineWidth _ (4)
            OutlineColor _ ($model.color)
            Circle c (0, 0, $radius)
            radius =:> c.r
        }

        idle -> animate (model.start_locate, radius_anim.start)
        animate -> idle (model.stop_locate, radius_anim.abort)
    }

}