use core
use base
use gui


_define_
TrapStatusSelector (Process _frame, Process _context)
{
    //TextPrinter tp

    DerefDouble tx (_context.ref_current_trap, "screen_translation/tx", DJNN_GET_ON_CHANGE)
    DerefDouble ty (_context.ref_current_trap, "screen_translation/ty", DJNN_GET_ON_CHANGE)

    Translation tr (0, 0)
    tx.value + 15 => tr.tx
    ty.value + 3 => tr.ty

    //DerefInt model_id (_context.ref_current_trap, "model/id", DJNN_GET_ON_CHANGE)
    DerefString model_state (_context.ref_current_trap, "model/state", DJNN_GET_ON_CHANGE)
    //"id: " + model_id.value + " -- state: " + model_state.value =:> tp.input

    Deref unknown_assignement (_context.ref_current_trap, "model/unknown_assignement")
    Deref identified_assignement (_context.ref_current_trap, "model/identified_assignement")
    Deref deactivated_assignement (_context.ref_current_trap, "model/deactivated_assignement")
    Deref delete_assignement (_context.ref_current_trap, "model/delete_assignement")


    svg = load_from_XML ("res/svg/trap_status_selector.svg")
    
    FSM fsm {
        State hidden

        State visible {
            bg << svg.bg
            m_unknown << svg.mask_unknown
            m_identified << svg.mask_identified
            m_deactivated << svg.mask_deactivated
            button_delete << svg.delete_btn

            //close_btn << svg.close_button
            //close_btn.close_mask.press -> _context.set_current_trap_to_null

            FSM fsm_status {
                State unknown {
                    r_unknown << svg.rect_unknown
                    r_unknown.press -> unknown_assignement.activation
                    r_unknown.press -> _context.set_current_trap_to_null
                }
                State identified {
                    r_identified << svg.rect_identified
                    r_identified.press -> identified_assignement.activation
                    r_identified.press -> _context.set_current_trap_to_null
                }
                State deactivated {
                    r_deactivated << svg.rect_deactivated
                    r_deactivated.press -> deactivated_assignement.activation
                    r_deactivated.press -> _context.set_current_trap_to_null
                }
                {unknown,  identified} -> deactivated (m_deactivated.enter)
                {identified, deactivated} -> unknown (m_unknown.enter)
                {deactivated, unknown} -> identified (m_identified.enter)
            }
        
            t_unknown << svg.unknown
            t_identified << svg.identified
            t_deactivated << svg.deactivated

            button_delete.rect_delete.press -> delete_assignement.activation
            button_delete.rect_delete.press -> _context.set_current_trap_to_null
        }
        hidden -> visible (_context.is_null_current_trap.false)
        visible -> hidden (_context.is_null_current_trap.true)
    }
    // FIXME: works only if menu is closed, then opened again
    model_state.value =:> fsm.visible.fsm_status.initial
}
