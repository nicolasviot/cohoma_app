use core
use gui
use display
use base

_define_
SubLayer (Process _model)
{
    model aka _model

    TextPrinter tp

    Switch switch (visible) {
        Component hidden {
            "sub layer " +_model.name + " is hidden" =: tp.input
        }
        Component visible {
            "sub layer " + _model.name + " is visible" =: tp.input
        }
    }
    _model.is_visible =:> switch.state
}
