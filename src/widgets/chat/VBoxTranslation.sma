use core
use base
use display
use gui

import gui.widgets.VBox

_define_
VBoxTranslation (Process container) inherits VBox (container) {
    //dump this.g
    addChildrenTo this.g {
        FillColor fc (255,255,0)
        Rectangle bg (0,0,0,0,0,0)
        Translation tr(0,0)
    }
    //dump this.g
    //moveChild this.g.fc < this.g.items
    //dump this.g
    //moveChild this.g.bg < this.g.items
    //dump this.g
    moveChild this.g.tr < this.g.items
    //dump this
    this.{x,y,width,height} =:> this.g.bg.{x,y,width,height}
    TextPrinter tp
    //toString(this.g.bg.width) + " " + toString(this.g.bg.height) =:> tp.input
    TextPrinter tp2
    //toString(this.min_width) + " " + toString(this.min_height) =:> tp2.input
}
