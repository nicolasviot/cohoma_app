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
            bg << svg.bg
            header << svg.header
            top << svg.top
            bottom << svg.bottom
        }
        hidden -> visible (_context.is_null_selected_trap.false)
        visible -> hidden (_context.is_null_selected_trap.true)
    }
    
}
