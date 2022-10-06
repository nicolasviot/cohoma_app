use core
use base
use gui


_define_
StatusSelector (Process f, Process _context)
{
    Translation pos (0, 0)
    x aka pos.tx
    y aka pos.ty

    cur_wpt aka _context.entered_wpt


    DerefString wp_status (cur_wpt, "usage_status", DJNN_GET_ON_CHANGE)
    Deref wp_leave (cur_wpt, "leave")
    Deref wp_press (cur_wpt, "right_press")
    Spike show
    Spike hide
    String status ("default")
    cur_wpt->show

    AssignmentSequence set_status (1) {
        status =: wp_status.value
    }

    svg = loadFromXML ("res/svg/status_selector.svg")

    FSM ctrl_visibility {
        State idle

        State hidden {
            f.move.x =:> x
            f.move.y =:> y
        }
        
        State visible {
            bg << svg.bg
            m_start << svg.mask_start
            m_end << svg.mask_end
            m_mandatory << svg.mask_mandatory
            m_forced << svg.mask_forced
            m_default << svg.mask_default
            Spike press
            FSM fsm_status {
                State default {
                    r_default << svg.rect_default
                    "default" =: status
                    r_default.press->press
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
                    "mandatory" =: status
                    r_mandatory.press->press
                }
                State forced {
                    r_forced << svg.rect_forced
                    "forced" =: status
                    r_forced.press->press
                }
                {start, mandatory, forced, end}->default (m_default.enter)
                {default, start, end, forced}->mandatory (m_mandatory.enter)
                {default, start, end, mandatory}->forced (m_forced.enter)
                {default, end, mandatory, forced}->start (m_start.enter)
                {default, start, mandatory, forced}->end (m_end.enter)
                
            }
            t_start << svg.start
            t_end << svg.end
            t_mandatory << svg.mandatory 
            t_forced << svg.forced
            t_default << svg.default
        }
        idle->hidden (cur_wpt)
        hidden->idle (wp_leave.activation)
        hidden->visible (wp_press.activation)
        visible->hidden (visible.press, set_status)
        visible->hidden (f.press)
    }
    wp_status.value =:> ctrl_visibility.visible.fsm_status.initial 
}
