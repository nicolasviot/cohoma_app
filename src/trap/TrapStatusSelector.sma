use core
use base
use gui


_define_
TrapStatusSelector (Process trap) {

    Spike press 
    Spike hide

    svg = loadFromXML ("res/svg/trap_status_selector.svg")
    
    FSM set_State_Menu{
        State hidden{

        }
        State visible{
            Translation _ (35, 15) //position right center from the trap
            bg << svg.bg
            m_unknown << svg.mask_unknown
            m_identified << svg.mask_identified
            m_deactivated << svg.mask_deactivated
            button_delete << svg.delete_btn
            close_btn << svg.close_button
            close_btn.close_mask.press -> hide

            FSM fsm_status{
                State unknown{
                    r_unknown << svg.rect_unknown
                    r_unknown.press -> trap.unknown_assignement
                    r_unknown.press -> trap.state_manually_updated
                }
                State identified{
                    r_identified << svg.rect_identified
                    r_identified.press -> trap.identified_assignement
                    r_identified.press -> trap.state_manually_updated
                }
                State deactivated{
                    r_deactivated << svg.rect_deactivated
                    r_deactivated.press -> trap.deactivated_assignement
                    r_deactivated.press -> trap.state_manually_updated
                }
                {unknown,  identified} -> deactivated (m_deactivated.enter)
                {identified, deactivated} -> unknown (m_unknown.enter)
                {deactivated, unknown} -> identified (m_identified.enter)
            }
        
            t_unknown << svg.unknown
            t_identified << svg.identified
            t_deactivated << svg.deactivated

            button_delete.rect_delete.press -> trap.ask_delete
            button_delete.rect_delete.press -> trap.delete_assignement
    
        }
        hidden -> visible (press)
        visible -> hidden (press)
        visible -> hidden (hide)
    } 
    trap.state =:> set_State_Menu.visible.fsm_status.initial 
}
