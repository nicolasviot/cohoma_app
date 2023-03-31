use core
use base
use gui

_native_code_
%{
    
%}


_define_
TrapForm (Process _context, Process _frame)
{
    //context aka _context

    TextPrinter tp

    Spike show
    Spike hide

    svg = load_from_XML_once ("res/svg/trap_form.svg")

    Double init_width ($svg.bg.width)
    Double init_height ($svg.bg.height)

    Translation tr (0, $_context.TOP_BAR_HEIGHT + 3)
    _frame.width - _context.RIGHT_PANEL_WIDTH - (init_width + 3) =:> tr.tx

    /*AssignmentSequence update_position (1) {
        _context.pointer_lat =: lat
        _context.pointer_lon =: lon

        (lon2px ($lon, $_map.zoomLevel) - _map.t0_x) =: tr.tx
        _map.t0_y - lat2py ($lat, $_map.zoomLevel) =: tr.ty
    }

    show -> update_position*/

    DerefString str_id (_context.ref_selected_trap, "str_id", DJNN_GET_ON_CHANGE)
    DerefString detection_time (_context.ref_selected_trap, "detection_time", DJNN_GET_ON_CHANGE)
    DerefString detection_robot_name (_context.ref_selected_trap, "detection_robot_name", DJNN_GET_ON_CHANGE)

    DerefBool is_identified (_context.ref_selected_trap, "identified", DJNN_GET_ON_CHANGE)

    DerefString identification_time (_context.ref_selected_trap, "identification_time", DJNN_GET_ON_CHANGE)
    DerefString identification_robot_name (_context.ref_selected_trap, "identification_robot_name", DJNN_GET_ON_CHANGE)
    DerefString nature (_context.ref_selected_trap, "nature", DJNN_GET_ON_CHANGE)
    DerefString misc (_context.ref_selected_trap, "misc", DJNN_GET_ON_CHANGE)
    DerefInt identifier (_context.ref_selected_trap, "identifier", DJNN_GET_ON_CHANGE)

    DerefString deactivation_time (_context.ref_selected_trap, "deactivation_time", DJNN_GET_ON_CHANGE)
    DerefBool is_remote (_context.ref_selected_trap, "remote", DJNN_GET_ON_CHANGE)
    DerefString remote_code (_context.ref_selected_trap, "remote_code", DJNN_GET_ON_CHANGE)
    DerefBool is_contact (_context.ref_selected_trap, "contact", DJNN_GET_ON_CHANGE)
    DerefString str_contact_mode (_context.ref_selected_trap, "str_contact_mode", DJNN_GET_ON_CHANGE)
    DerefString contact_code (_context.ref_selected_trap, "contact_code", DJNN_GET_ON_CHANGE)

    DerefBool is_active (_context.ref_selected_trap, "active", DJNN_GET_ON_CHANGE)


    FSM fsm {
        State hidden

        State visible {

            bg << svg.bg

            // header / detection
            header << svg.header

            "#" + str_id.value =:> header.txt_id_icare.text
            "détecté par " + detection_robot_name.value =:> header.txt_detection_robot.text
            detection_time.value =:> header.txt_detection_time.text

            Switch switch_identified (false) {
                Component false {
                    35 =: bg.height
                }

                Component true {
                    init_height =: bg.height

                    // top part / identification
                    top << svg.top

                    identifier.value =:> top.txt_identifier.text
                    "identifié par " + identification_robot_name.value =:> top.txt_identif_robot.text
                    identification_time.value =:> top.txt_identif_time.text
                    nature.value =:> top.txt_nature.text
                    misc.value =:> top.txt_misc.text
                    
                    // bottom part / deactivation
                    bottom << svg.bottom.global

                    deactivation_time.value =:> bottom.txt_deactiv_time.text

                    Deref deactivate (_context.ref_selected_trap, "deactivate")
                    Deref activate (_context.ref_selected_trap, "activate")

                    // Active
                    Switch switch_active (true) {
                        Component true {
                            130.2 =: bottom.toggle.toggle_handle.cx

                            bottom.toggle.toggle_bg.press -> deactivate.activation
                        }

                        Component false {
                            140.2 =: bottom.toggle.toggle_handle.cx

                            bottom.toggle.toggle_bg.press -> activate.activation
                        }
                    }
                    is_active.value =:> switch_active.state

                    is_remote.value ? "OUI" : "NON" =:> bottom.txt_bool_distance.text
                    is_contact.value ? "OUI" : "NON" =:> bottom.txt_bool_contact.text

                    // Remote
                    Switch switch_remote (false) {
                        Component false

                        Component true {
                            remote << svg.bottom.remote
                            remote_code.value =:> remote.txt_code_remote.text
                        }
                    }
                    is_remote.value =:> switch_remote.state

                    // Contact
                    Switch switch_contact (false) {
                        Component false

                        Component true {
                            contact << svg.bottom.contact
                            str_contact_mode.value =:> contact.txt_mode_contact.text
                            contact_code.value =:> contact.txt_code_contact.text
                        }
                    }
                    is_contact.value =:> switch_contact.state
                }
            }
            // FIXME: not taken into account if "DerefBool is_identified" in inside "State visible {"
            is_identified.value =:> switch_identified.state
            //is_identified.value + " - " + switch_identified.state =:> tp.input
        }
        hidden -> visible (_context.is_null_selected_trap.false)
        visible -> hidden (_context.is_null_selected_trap.true)
    }
    
    //"FSM: " + fsm.state =:> tp.input
}
