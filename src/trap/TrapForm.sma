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

    Translation tr (0, $_context.TOP_BAR_HEIGHT + 3)
    _frame.width - _context.RIGHT_PANEL_WIDTH - (svg.bg.width + 3) =:> tr.tx

    /*AssignmentSequence update_position (1) {
        _context.pointer_lat =: lat
        _context.pointer_lon =: lon

        (lon2px ($lon, $_map.zoomLevel) - _map.t0_x) =: tr.tx
        _map.t0_y - lat2py ($lat, $_map.zoomLevel) =: tr.ty
    }

    show -> update_position*/

    FSM fsm {
        State hidden

        State visible {
            DerefString str_id (_context.ref_selected_trap, "str_id", DJNN_GET_ON_CHANGE)
            DerefString detection_time (_context.ref_selected_trap, "detection_time", DJNN_GET_ON_CHANGE)
            DerefString detection_robot_name (_context.ref_selected_trap, "detection_robot_name", DJNN_GET_ON_CHANGE)

            DerefString identification_time (_context.ref_selected_trap, "identification_time", DJNN_GET_ON_CHANGE)
            DerefString identification_robot_name (_context.ref_selected_trap, "identification_robot_name", DJNN_GET_ON_CHANGE)
            DerefString nature (_context.ref_selected_trap, "nature", DJNN_GET_ON_CHANGE)
            DerefString misc (_context.ref_selected_trap, "misc", DJNN_GET_ON_CHANGE)
            DerefInt identifier (_context.ref_selected_trap, "identifier", DJNN_GET_ON_CHANGE)

            bg << svg.bg

            // header
            header << svg.header

            "#" + str_id.value =:> header.txt_id_icare.text
            "détecté par " + detection_robot_name.value =:> header.txt_detection_robot.text
            detection_time.value =:> header.txt_detection_time.text

            // top part
            top << svg.top

            identifier.value =:> top.txt_identifier.text
            "identifié par " + identification_robot_name.value =:> top.txt_identif_robot.text
            identification_time.value =:> top.txt_identif_time.text
            nature.value =:> top.txt_nature.text
            misc.value =:> top.txt_misc.text
            
            // bottom part
            bottom << svg.bottom
        }
        hidden -> visible (_context.is_null_selected_trap.false)
        visible -> hidden (_context.is_null_selected_trap.true)
    }
    
}
