use core
use gui
use display
use base
use files

_define_
ClockComponent (Process _frame)
{
    //afficher l'heure
    WallClock wc
    "%H:%M:%S" =: wc.format   // exemple=  "%H:%M:%S", "%Hh%Mm%Ss"

    FillColor white (#FFFFFF)
    //FontWeight _ (75)
    FontSize _ (5, 14)
    Text text_clock (10, -5, "default")
    wc.state_text =:> text_clock.text
    
    // il faut une source trigger
    // FIXME: 250ms is sufficient (vs 100ms) ?
    Clock clock_trigger (250)
    clock_trigger.tick -> wc.state_text

    WallClock wc_name

    //FileWriter fw ("logs/Log.log")
    FileWriter fw ("logs/Log_" + wc_name.state_text + ".log")
    wc.state_text + " - Application start\n" =: fw.input

    "Click at (" + _frame.press.x + ", " + _frame.press.y + ")\n" => fw.input

    FileWriter fw_console ("logs/Log_console_" + wc_name.state_text + ".log")
    wc.state_text + " - Application start\n" =: fw_console.input
}
