use core
use gui
use base

//import behavior.DraggableItemWithRadius
import behavior.NotDraggableItemWithRadius

_native_code_
%{
    #include <iostream>
%}


_define_
//Trap (Process _map, Process _context, Process _model, Process _svg_info, Process _svg_remotely_icon, Process _svg_contact_icon)
Trap (Process _map, Process _context, Process _model)
{
    //map aka _map
    context aka _context
    model aka _model

    TextPrinter tp

    Spike to_delete
    _model.deleted.true -> to_delete

    Translation screen_translation (0, 0)

    // Encapsulates content to prevent opacities interferences with menu and localization
    Component content {
    
        // FIXME: use a switch to really deactivate it. Or remove from djnn tree
        FillOpacity global_opacity (1)
        
        Switch sw_bg_circle (visible) {
            Component visible {
                OutlineOpacity o_op (1.0)
                OutlineColor o_col ($_context.TRAP_COLOR)
                OutlineWidth o_w (2)
                //_model.is_selected ? _context.SELECTION_COLOR : _context.TRAP_COLOR =:> o_col.value

                // Circle (linked with radius)
                FillColor f_col ($_context.TRAP_COLOR)
                FillOpacity circle_opacity (0.2)
                Circle c (0, 0, 50)
                //_model.radius =:> c.r     // made in the DraggableItemWithRadius to respect map zoom (meters --> pixels)
            }

            Component hidden
        }

        Component losange {
            Rotation rot (45, 0, 0)

            Component bg {
                OutlineOpacity o_op (0.0)
                OutlineColor o_col (#000000)
                OutlineWidth o_w (1)

                FillColor f_col ($_context.TRAP_COLOR)
                Rectangle rect (-$_context.TRAP_SIZE / 2, -$_context.TRAP_SIZE / 2, $_context.TRAP_SIZE, $_context.TRAP_SIZE)
            }

            //Component fg {
                OutlineOpacity o_op (0.0)
                _model.is_selected ? 1.0 : 0.0 =:> o_op.a

                OutlineColor o_col ($_context.SELECTION_COLOR)
                OutlineWidth o_w (4)
                NoFill _
                Rectangle feedback (-$_context.TRAP_SIZE / 2, -$_context.TRAP_SIZE / 2, $_context.TRAP_SIZE, $_context.TRAP_SIZE)
            //}
        }
        // for interactions
        picking aka losange.bg.rect
        
        // Label
        FillColor _ (#000000)

        FontSize _ (0, 10)
        TextAnchor _ (DJN_MIDDLE_ANCHOR)
        //FontWeight fwg (DJN_BOLD) (DJN_NORMAL)
        //FontStyle fst (DJN_ITALIC_FONT) (DJN_NORMAL_FONT)
        Text label_trap_id (0, 5, "")
        "#" + toString(_model.str_id) =: label_trap_id.text


        // state switch
        /*Switch trap_state_switch (unknown) {
            Component unknown {
                // set radius to maximum possible radius
                50 =: _model.radius

                // fill in red
                240 =: fill_col.r
                0.1 =: outline_op.a
                1 =: global_opacity.a

                "#" + _model.id =:> label_trap_id.text             
            }

            Component identified {
                // set circle radius to 20
                20 =: _model.radius

                // fill in red
                240 =: fill_col.r
                1.0 =: outline_op.a
                1 =: global_opacity.a

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
                // set circle radius to zero
                0 =: c.r

                //fill in grey
                100 =: fill_col.r
                0.1 =: outline_op.a
                0.3 =: global_opacity.a
                
                "#" + _model.id =:> label_trap_id.text
            }
        }
        _model.state =:> trap_state_switch.state*/

        Switch switch (st_detected) {
            Component st_detected {
                1.0 =: global_opacity.a
                0.0 =: losange.o_op.a
                "visible" =: sw_bg_circle.state
            }

            Component st_identified {
                1.0 =: global_opacity.a
                1.0 =: losange.o_op.a
                "visible" =: sw_bg_circle.state
                _model.identifier =: label_trap_id.text
            }

            Component st_deactivated {
                0.4 =: global_opacity.a
                "hidden" =: sw_bg_circle.state
                //#FF6200 =: losange.bg.f_col.value
            }

            Component st_hidden { // FIXME: use a switch to really deactivate it. Or remove from djnn tree
                0.01 =: global_opacity.a
                "hidden" =: sw_bg_circle.state
            }

            Component st_deleted
        }
        _model.fsm.state =:> switch.state
        

        // Update the position via "screen_translation" in function of lat/lon and current zoom level
        // Allow to drag via "picking"
        //DraggableItemWithRadius draggable_item (_map, _context, _model.lat, _model.lon, _model.radius, screen_translation.tx, screen_translation.ty, picking, _context.frame_released, sw_bg_circle.visible.c.r)
        NotDraggableItemWithRadius not_draggable_item (_map, _model.lat, _model.lon, _model.radius, screen_translation.tx, screen_translation.ty, sw_bg_circle.visible.c.r)
    }


    ///////TRAP INFO OVERLAY ON HOVER //////
        
    /*FSM info_overlay_FSM {
        State idle

        State visible {
            Translation _ (0, -15)
            info << clone (_svg_info.trap_info)

            _model.nature =:> info.description_text.text
            _model.code =:> info.code_text.text
            _model.hazard =:> info.hazard_text.text
            "#" + toString(_model.id) =:> info.id_text.text
            _model.radius =:> info.radius_text.text
            _model.remotely_deactivate ? (_model.contact_deactivate ? "Remote/Contact" : "Remote") : (_model.contact_deactivate ? "Contact" : "...") =:> info.deactivate_text.text
            _model.contact_text =:> info.contact_text.text
        }
        idle -> visible (content.picking.enter)
        visible -> idle (content.picking.leave)
    }*/

    content.picking.left.release -> na_click:(this) {
        if (this.model.is_selected)
        {
            // FIXME 1: we can't activate the AssignmentSequence set_selected_trap_to_null
            //notify this.context.set_selected_trap_to_null
            // FIXME 2: so we had to create a Spike "spike_to_set_selected_trap_to_null"
            notify this.context.spike_to_set_selected_trap_to_null
            this.model.is_selected = false
        }
        else
        {
            if (!this.context.is_null_selected_trap) {
                previous_selected_trap = getRef (this.context.ref_selected_trap)
                previous_selected_trap.is_selected = false
            }

            setRef (this.context.ref_selected_trap, this.model)
            this.model.is_selected = true
        }
    }

    /*content.picking.right.press -> {
        this =: _context.ref_current_trap
    }*/
    
}