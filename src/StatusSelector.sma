use core
use base
use gui


_define_
StatusSelector (Process f, Process _manager) {
    Translation pos (0, 0)
    x aka pos.tx
    y aka pos.ty

    cur_wpt aka _manager.entered_wpt


    DerefString wp_status (cur_wpt, "usage_status", DJNN_GET_ON_CHANGE)
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
            m_start << svg.mask_start
            m_end << svg.mask_end
            m_mandatory << svg.mask_mandatory
            m_forbidden << svg.mask_forbidden
            m_usable << svg.mask_usable
            Spike press
            FSM fsm_status {
                State default
                State usable {
                    r_usable << svg.rect_usable
                    "usable" =: status
                    r_usable.press->press
                }
                State start {
                    r_start << svg.rect_start
                    "start" =: status
                    r_start.press->press
                }
                State end {
                    r_end << svg.rect_end
                    "end" =: status
                    r_end.press->press
                }
                State mandatory {
                    r_mandatory << svg.rect_mandatory
                    "compulsory" =: status
                    r_mandatory.press->press
                }
                State forbidden {
                    r_forbidden << svg.rect_forbidden
                    "forbidden" =: status
                    r_forbidden.press->press
                }
                {default, usable, start, end, forbidden}->mandatory (m_mandatory.enter)
                {default, start, end, mandatory, forbidden}->usable (m_usable.enter)
                {default, usable, start, end, mandatory}->forbidden (m_forbidden.enter)
                {default, usable, end, mandatory, forbidden}->start (m_start.enter)
                {default, usable, start, mandatory, forbidden}->end (m_end.enter)
                
            }
            t_start << svg.start
            t_end << svg.end
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
    wp_status.value =:> ctrl_visibility.visible.fsm_status.initial 
}
