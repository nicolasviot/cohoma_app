use core
use gui
use display
use base
use files

_define_
ClockComponent (double _x, double _y, Process frame){
Translation t(_x, _y)

    //afficher l'heure
    WallClock wc 
    wc.format = "%H:%M:%S"  // exemple=  "%H:%M:%S", "%Hh%Mm%Ss" 
    FillColor white (255, 255, 255)
    Text text_clock (20, 20, "default")
    //wc.state_text =:> text_clock.text
    // il faut une source trigger
    Clock clock_trigger (100)
    //clock_trigger.tick -> wc.state_text

    FileWriter fw ("toto.log")
    "Log_" + wc.state_text =: fw.filename
    wc.state_text + " - Application start" =: fw.input
    FileWriter fw_console("console.log")
    "Log_console_" + wc.state_text =: fw_console.filename



    "Click at (" + frame.press.x + ", " + frame.press.y + ")\n" => fw.input

    //wc.state_text + " - toto\n" => fw.input
}
