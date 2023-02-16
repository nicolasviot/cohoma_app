use core
use base
use gui

import gui.widgets.HBox
import gui.widgets.VBox
import gui.widgets.PushButton
import gui.widgets.Label

import gui.widgets.StandAlonePushButton
import gui.widgets.UITextField

import widgets.chat.Bubble


_define_
Chat (Process frame) {
    // FIXME cannot create an empty Composite
    
    mouseTracking = 1

    FillColor _(White)

    VBox vbox (frame)
        VBox msgs_ (vbox)
        HBox msg_send (vbox)
            UITextField msg_
            msg_.preferred_width = 200
            PushButton send_ ("send")
            addChildrenTo msg_send.items {
                msg_, send_
            }
        addChildrenTo vbox.items {
            msgs_,
            msg_send
        }
    
    msgs aka this.vbox.items.[1].items
    msg  aka this.vbox.items.[2].items.[1]
    send aka this.vbox.items.[2].items.[2]

    //msg.field.content.text != "" =:> send.enabled
    
    send.click -> send_msg: (this) {
        addChildrenTo this.msgs {
            Bubble b ("toto")
            b.ui.text = toString(this.msg.field.content.text)
                //l.ui.anchor = 
                //l.ui.width =:> bg.width
                //l.ui.height =:> bg.height
            //}
        }
    }
    send_msg -> msg.clear

}