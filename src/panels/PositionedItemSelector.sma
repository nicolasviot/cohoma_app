use core
use base
use gui

_native_code_
%{
    #include "cpp/coords-utils.h"
%}


_define_
PositionedItemSelector (Process _map, Process _context)
{
    TextPrinter tp

    Double lat (0)
    Double lon (0)

    Spike show
    Spike hide

    Spike add_trap
    Spike add_message
    Spike add_goto

    add_trap -> {
        "Add a new trap at " + lat + " " + lon =: tp.input
    }
    add_trap -> hide

    add_message -> {
        "Add a new message at " + lat + " " + lon =: tp.input
    }
    add_message -> hide

    add_goto -> {
        "Add a go to at " + lat + " " + lon =: tp.input
    }
    add_goto -> hide

    Translation tr (0, 0)

    AssignmentSequence update_position (1) {
        _context.pointer_lat =: lat
        _context.pointer_lon =: lon

        (lon2px ($lon, $_map.zoomLevel) - _map.t0_x) =: tr.tx
        _map.t0_y - lat2py ($lat, $_map.zoomLevel) =: tr.ty
    }

    show -> update_position

    svg = load_from_XML_once ("res/svg/positioned_selector.svg")

    FSM fsm {
        State hidden

        State visible {
            bg << svg.bg

            btn_trap << svg.trap
            //0.0 =: btn_trap.rect_trap.fill\-opacity.a

            btn_message << svg.message
            //0.0 =: btn_message.rect_message.fill\-opacity.a

            btn_goto << svg.goto
            //0.0 =: btn_goto.rect_goto.fill\-opacity.a

            FSM fsm_options {
                State idle {
                    0.0 =: btn_trap.rect_press_trap.fill\-opacity.a
                    0.0 =: btn_message.rect_press_message.fill\-opacity.a
                    0.0 =: btn_goto.rect_press_goto.fill\-opacity.a
                }

                State hover_trap {
                    0.2 =: btn_trap.rect_press_trap.fill\-opacity.a
                }
                State press_trap {
                    1.0 =: btn_trap.rect_press_trap.fill\-opacity.a
                }

                State hover_message {
                    0.2 =: btn_message.rect_press_message.fill\-opacity.a
                }
                State press_message {
                    1.0 =: btn_message.rect_press_message.fill\-opacity.a
                }

                State hover_goto {
                    0.2 =: btn_goto.rect_press_goto.fill\-opacity.a
                }
                State press_goto {
                    1.0 =: btn_goto.rect_press_goto.fill\-opacity.a
                }

                idle -> hover_trap (btn_trap.rect_trap.enter)
                hover_trap -> idle (btn_trap.rect_trap.leave)
                hover_trap -> press_trap (btn_trap.rect_trap.press)
                press_trap -> idle (btn_trap.rect_trap.release, add_trap)

                idle -> hover_message (btn_message.rect_message.enter)
                hover_message -> idle (btn_message.rect_message.leave)
                hover_message -> press_message (btn_message.rect_message.press)
                press_message -> idle (btn_message.rect_message.release, add_message)

                idle -> hover_goto (btn_goto.rect_goto.enter)
                hover_goto -> idle (btn_goto.rect_goto.leave)
                hover_goto -> press_goto (btn_goto.rect_goto.press)
                press_goto -> idle (btn_goto.rect_goto.release, add_goto)
            }
        }
        hidden -> visible (show)
        visible -> hidden (hide)
    }
    
}
