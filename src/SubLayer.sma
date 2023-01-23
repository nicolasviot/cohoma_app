use core
use gui
use display
use base

_define_
SubLayer (Process _model)
{
    model aka _model

    TextPrinter tp

    Switch switch (true) {
        Component true {
            "sub layer " + _model.name + " is visible" =: tp.input
        }
        Component false {
            "sub layer " +_model.name + " is hidden" =: tp.input
        }
    }
    _model.is_visible =:> switch.state
}
