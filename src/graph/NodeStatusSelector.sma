use core
use base
use gui


_define_
NodeStatusSelector (Process _frame, Process _context)
{
    TextPrinter tp

    String selected_status ("default")

    DerefDouble tx (_context.ref_current_node, "screen_translation/tx", DJNN_GET_ON_CHANGE)
    DerefDouble ty (_context.ref_current_node, "screen_translation/ty", DJNN_GET_ON_CHANGE)

    Translation tr (0, 0)
    tx.value + 3 => tr.tx
    ty.value => tr.ty

    // FIXME: replace by model when GraphNode will be linked to its model
    //DerefInt model_id (_context.ref_current_node, "model/id", DJNN_GET_ON_CHANGE)
    DerefInt model_id (_context.ref_current_node, "id", DJNN_GET_ON_CHANGE)
    //DerefString model_status (_context.ref_current_node, "model/status", DJNN_GET_ON_CHANGE)
    DerefString model_status (_context.ref_current_node, "usage_status", DJNN_GET_ON_CHANGE)
    //"id: " + model_id.value + " -- status: " + model_status.value =:> tp.input


    AssignmentSequence set_status_to_model (1) {
        selected_status =: model_status.value
    }
    set_status_to_model -> _context.set_current_node_to_null


    svg = loadFromXML ("res/svg/status_selector.svg")

    FSM fsm {
        State hidden
        
        State visible {
            bg << svg.bg
            m_start << svg.mask_start
            m_end << svg.mask_end
            m_mandatory << svg.mask_mandatory
            m_forced << svg.mask_forced
            m_default << svg.mask_default
            
            FSM fsm_status {
                State default {
                    r_default << svg.rect_default
                    "default" =: selected_status
                    r_default.press -> set_status_to_model
                }
                State start {
                    r_start << svg.rect_start
                    "start" =: selected_status
                    r_start.press -> set_status_to_model
                }
                State end {
                    r_end << svg.rect_end
                    "end" =: selected_status
                    r_end.press -> set_status_to_model
                }
                State mandatory {
                    r_mandatory << svg.rect_mandatory
                    "mandatory" =: selected_status
                    r_mandatory.press -> set_status_to_model
                }
                State forced {
                    r_forced << svg.rect_forced
                    "forced" =: selected_status
                    r_forced.press -> set_status_to_model
                }
                {start, mandatory, forced, end} -> default (m_default.enter)
                {default, start, end, forced} -> mandatory (m_mandatory.enter)
                {default, start, end, mandatory} -> forced (m_forced.enter)
                {default, end, mandatory, forced} -> start (m_start.enter)
                {default, start, mandatory, forced} -> end (m_end.enter)
                
            }
            t_start << svg.start
            t_end << svg.end
            t_mandatory << svg.mandatory 
            t_forced << svg.forced
            t_default << svg.default
        }
        hidden -> visible (_context.is_null_current_node.false)
        visible -> hidden (_context.is_null_current_node.true)
    }
    // FIXME: works only if menu is closed, then opened again
    model_status.value =:> fsm.visible.fsm_status.initial 
}
