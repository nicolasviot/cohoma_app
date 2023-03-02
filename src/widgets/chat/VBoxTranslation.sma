use core
use base
use display
use gui

import gui.widgets.VBox

_define_
VBoxTranslation (Process container) inherits VBox (container) {
    addChildrenTo this.g {
        Translation tr(0,0)
    }
    moveChild this.g.tr < this.g.items
}
