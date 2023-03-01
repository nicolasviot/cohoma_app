use core
use base
use gui

import gui.widgets.HBox
import gui.widgets.VBox
import gui.widgets.PushButton
import gui.widgets.Label
//import gui.widgets.StandAlonePushButton
import gui.widgets.UITextField
import widgets.Scrollbar

import widgets.chat.Bubble


_define_
Chat (Process frame) {
    // FIXME cannot create an empty Composite
    
    mouseTracking = 1

    //RectangleClip cpr_(0,0,800,800) // does not really work in a responsive interface...

    FillColor _(White)

    VBox main_box (frame)
        HBox conversation_box (main_box)
            VBox conversation_ (conversation_box)
            Scrollbar sb_ (frame)
            addChildrenTo conversation_box.items {
                conversation_, sb_
            }

        HBox new_msg_ (main_box)
            UITextField edit_
            edit_.preferred_width = 200
            PushButton send_ ("send")
            addChildrenTo new_msg_.items {
                edit_, send_
            }
        addChildrenTo main_box.items {
            conversation_box,
            new_msg_
        }
    
    conversation aka this.main_box.items.[1].items.[1].items
    sb aka this.main_box.items.[1].items.[2]
    //Scrollbar sb(frame)
    edit  aka this.main_box.items.[2].items.[1]
    send aka this.main_box.items.[2].items.[2]
    new_msg aka this.main_box.items.[2]

    10 =: sb.preferred_width
    100 =: sb.preferred_height
    2 =: sb.h_alignment
    2 =: new_msg.v_alignment


    //sb.preferred_width =: sb.width
    //sb.preferred_height =: sb.height

    //msg.field.content.text != "" =:> send.enabled
    
    //vbox.{x,y,width,height} =:> cpr_.{x,y,width,height}


    send.click -> send_msg: (this) {
        addChildrenTo this.conversation {
            Bubble b ("toto")
            b.ui.text = toString(this.edit.field.content.text)
                //l.ui.anchor = 
                //l.ui.width =:> bg.width
                //l.ui.height =:> bg.height
            //}
        }
    }
    send_msg -> edit.clear

}