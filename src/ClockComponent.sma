use core
use gui
use display
use base
use files

_define_
ClockComponent (double _x, double _y, Process frame) {
    Translation t(_x, _y)

    //afficher l'heure
    WallClock wc
    "%H:%M:%S" =: wc.format   // exemple=  "%H:%M:%S", "%Hh%Mm%Ss"

    FillColor white (255, 255, 255)
    Text text_clock (20, 20, "default")
    wc.state_text =:> text_clock.text
    
    // il faut une source trigger
    // FIXME: 250ms is sufficient (vs 100ms) ?
    Clock clock_trigger (250)
    clock_trigger.tick -> wc.state_text

    WallClock wc_name

    //FileWriter fw ("logs/Log.log")
    FileWriter fw ("logs/Log_" + wc_name.state_text + ".log")
    wc.state_text + " - Application start\n" =: fw.input

    "Click at (" + frame.press.x + ", " + frame.press.y + ")\n" => fw.input

    FileWriter fw_console ("logs/Log_console_" + wc_name.state_text + ".log")
    wc.state_text + " - Application start\n" =: fw_console.input
}
