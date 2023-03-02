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
import widgets.chat.VBoxTranslation

_define_
Chat (Process frame) {
    // FIXME cannot create an empty Composite
    
    mouseTracking = 1

    //RectangleClip cpr_(0,0,800,800) // does not really work in a responsive interface...

    FillColor _(White)

    VBox main_box (frame)
        HBox conversation_box (main_box)
            VBoxTranslation conversation_ (conversation_box)
                // here: this is where messages are stored
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
    conversation_tr aka this.main_box.items.[1].items.[1].g.tr
    sb aka this.main_box.items.[1].items.[2]
    //Scrollbar sb(frame)
    edit  aka this.main_box.items.[2].items.[1]
    send aka this.main_box.items.[2].items.[2]
    new_msg aka this.main_box.items.[2]

    main_box.preferred_width = 100
    main_box.preferred_height = 100
    
    0 =: sb.model.low
    1 =: sb.model.high

    10 =: sb.preferred_width
    100 =: sb.preferred_height
    2 =: sb.h_alignment
    2 =: new_msg.v_alignment


    //sb.preferred_width =: sb.width
    //sb.preferred_height =: sb.height

    //deactivate (send)
    toString(edit.field.content.text) != "" -> send // FIXME there should be a disabled state
    toString(edit.field.content.text) == "" ->! send
    
    
    //vbox.{x,y,width,height} =:> cpr_.{x,y,width,height}

    // add message to conversation
    send.click -> add_msg: (this) {
        addChildrenTo this.conversation {
            Bubble b ("toto")
            b.ui.text = toString(this.edit.field.content.text)
            b.v_alignment = 2 // FIXME does not seem to be working
        }
    }
    add_msg -> edit.clear

    ((conversation.size > 2) && sb.model.low==0) -> {
        2.0 / conversation.size =: sb.model.high
    }
    ((conversation.size > 2) && sb.model.low>1) -> {
        sb.model.low + 2.0 / conversation.size =: sb.model.high
    }

    //_DEBUG_GRAPH_CYCLE_DETECT = 1

    // clip messages by disabling them
    sb.model.low -> (this) {
        int nb_items = this.conversation.size
        double v = 0
        double acc = 1. 
        acc = acc/nb_items
        for item : this.conversation {
            //printf("%f %f\n", acc, v)
            if ( v>= $this.sb.model.low && v <= $this.sb.model.high) {
                item.bg_color.b = 255
                //activate(item)
            } else {
                item.bg_color.b = 100
                //deactivate(item)
            }
            v += acc
        }
    }

    sb.model.low * 100 =:> conversation_tr.ty

}