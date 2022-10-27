use core
use gui
use base
use animation

import behavior.DraggableItemWithRadius
//import gui.animation.Animator
//import ros_node

_native_code_
%{
    #include <iostream>
%}


_define_
Trap (Process _map, Process _context, Process _model, Process _svg_info, Process _svg_remotely_icon, Process _svg_contact_icon)
{
    //map aka _map
    context aka _context
    model aka _model

    //TextPrinter tp

    Translation screen_translation (0, 0)

    // Encapsulates content to prevent opacities interferences with menu and localization
    Component content {
    
        OutlineOpacity trap_out_op (0)
        OutlineColor _ (0, 0, 0)
        OutlineWidth _ (2)

        FillOpacity global_opacity (1)
        // FIXME: use a switch to really deactivate it. Or remove from djnn tree
        _model.deleted.true -> {
            0.01 =: global_opacity.a
        }
        
        FillColor red (240, 0, 0)

        Component losange {
            Rotation rot (45, 0, 0)
            Rectangle rect (-15, -15, 30, 30)
        }
        // for interactions
        picking aka losange.rect

        NoOutline _

        // Circle (linked with radius)
        FillOpacity circle_opacity(0.1)
        Circle c (0, 0, 50)

        // always visible data : ID and deactivation mode 
        
        // Text for identification and information
        FillColor _ (0,0,0)
        FillOpacity text_opacity (3)
        1 / circle_opacity.a =:> text_opacity.a

        FontSize _ (0, 10)
        TextAnchor _ (DJN_MIDDLE_ANCHOR)
        Text label_trap_id (0, 5, "?")


        // state switch
        Switch trap_state_switch (unknown) {
            Component unknown {
                //fill in red
                240 =: red.r
                50 =: _model.radius //set radius to maximum possible radius
                0.1 =: trap_out_op.a
                1 =: global_opacity.a  
                "#" + _model.id =:> label_trap_id.text             
            }

            Component identified {
                240 =: red.r
                1 =: trap_out_op.a
                1 =: global_opacity.a

                20 =: _model.radius
                1 =: trap_out_op.a

                _model.str_id =:> label_trap_id.text

                // Add icons for active traps only
                Switch remotely_switch (false) {
                    Component true {
                        Translation _ (40, -5)
                        remote_icon << clone (_svg_remotely_icon.remotely_icon)
                    }
                    Component false
                }
                _model.remotely_deactivate =:> remotely_switch.state
    
                // Add icons for active traps only
                Switch contact_switch (false) {
                    Component true {
                        Translation _ (-10, -5)
                        contact_icon << clone (_svg_contact_icon.contact_icon)
                    }
                    Component false
                }
                _model.contact_deactivate =:> contact_switch.state
            }

            Component deactivated {
                0 =: c.r //set circle radius to zero
                0.1 =: trap_out_op.a
                0.3 =: global_opacity.a
                //fill in grey
                100 =: red.r
                "#" + _model.id =:> label_trap_id.text
            }
        }
        _model.state =:> trap_state_switch.state


        // Update the position via "screen_translation" in function of lat/lon and current zoom level
        // Allow to drag via "picking"
        DraggableItemWithRadius draggable_item (_map, _model.lat, _model.lon, _model.radius, screen_translation.tx, screen_translation.ty, picking, c.r)

    }


    ///////TRAP INFO OVERLAY ON HOVER //////
        
    FSM info_overlay_FSM {
        State idle

        State visible {
            Translation _ (0, -15)
            info << clone (_svg_info.trap_info)

            _model.description =:> info.description_text.text
            _model.code =:> info.code_text.text
            _model.hazard =:> info.hazard_text.text
            "#" + toString(_model.id) =:> info.id_text.text
            _model.radius =:> info.radius_text.text
            _model.remotely_deactivate ? (_model.contact_deactivate ? "Remote/Contact" : "Remote") : (_model.contact_deactivate ? "Contact" : "...") =:> info.deactivate_text.text
            _model.contact_text =:> info.contact_text.text
        }
        idle -> visible (content.picking.enter)
        visible -> idle (content.picking.leave)
    }


    content.picking.right.press -> {
        this =: _context.ref_current_trap
    }
    
}