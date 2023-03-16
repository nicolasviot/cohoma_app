use core
use gui
//use display
use base


_define_
TopBar (Process _context, Process _frame)
{
    OutlineWidth _ (1)
    OutlineColor _ (#E1E1E3)
    FillColor _ ($_context.DARK_GRAY)
    Rectangle bg (0, 0, 0, $_context.TOP_BAR_HEIGHT, 0, 0)
    _frame.width =:> bg.width

    FillColor white (#FFFFFF)
    //FontWeight _ (DJN_BOLD)
    //FontWeight _ (DJN_NORMAL)
    FontSize _ (5, 15) // 5 = pixel
    Text title (5, 20, "CoHoMa 2 - IHM Tactique")

    Text text_clock (200, 20, "..:..:..")
    _context.w_clock.state_text =:> text_clock.text
    
}
