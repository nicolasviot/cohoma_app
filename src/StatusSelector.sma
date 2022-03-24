use core
use base
use gui


_define_
StatusSelector (Process f, Process _manager) {
    Translation pos (0, 0)
    x aka pos.tx
    y aka pos.ty

    cur_wpt aka _manager.entered_wpt


    DerefString wp_status (cur_wpt, "usage_status", DJNN_IGNORE)
    Deref wp_leave (cur_wpt, "leave")
    Deref wp_press (cur_wpt, "right_press")
    Spike show
    Spike hide
    String status ("usable")
    cur_wpt->show

    AssignmentSequence set_status (1) {
        status =: wp_status.value
    }

    svg = loadFromXML ("res/svg/status_selector.svg")
    FSM ctrl_visibility {
        State idle
        State hidden {
            f.move.x + 5 =:> x
            f.move.y =:> y
        }
        State visible {
            bg << svg.bg
            m_mandatory << svg.mask_mandatory
            m_forbidden << svg.mask_forbidden
            m_usable << svg.mask_usable
            Spike press
            FSM fsm_status {
                State usable {
                    r_usable << svg.rect_usable
                    "usable" =: status
                    r_usable.press->press
                }
                State mandatory {
                    r_mandatory << svg.rect_mandatory
                    "mandatory" =: status
                    r_mandatory.press->press
                }
                State forbidden {
                    r_forbidden << svg.rect_forbidden
                    "forbidden" =: status
                    r_forbidden.press->press
                }
                {usable, forbidden}->mandatory (m_mandatory.enter)
                {forbidden, mandatory}->usable (m_usable.enter)
                {mandatory, usable}->forbidden (m_forbidden.enter)
                
            }
            t_mandatory << svg.mandatory 
            t_forbidden << svg.forbidden
            t_usable << svg.usable
        }
        idle->hidden (cur_wpt)
        hidden->idle (wp_leave.activation)
        hidden->visible (wp_press.activation)
        visible->hidden (visible.press, set_status)
        visible->hidden (f.press)
    }
}
