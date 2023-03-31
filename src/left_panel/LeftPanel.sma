use core
use gui
use base

import OperatorsList
import widgets.chat.Chat


_define_
LeftPanel (Process _context, Process _model_manager, Process _frame)
{
	//context aka _context

	Layer layer (0, $_context.TOP_BAR_HEIGHT, $_context.LEFT_PANEL_WIDTH, 1054) {

		Translation tr (0, $_context.TOP_BAR_HEIGHT)

		OutlineWidth _ (1)
    	OutlineColor _ (#E1E1E3)
		FillColor _ ($_context.DARK_GRAY)
		Rectangle bg (0, 0, $_context.LEFT_PANEL_WIDTH, 0, 0, 0)
		_frame.height =:> bg.height

		// List of operators (and their vehicles)
		OperatorsList operators_list (_context, _model_manager)

		// Chat to send (and receive) messages to others operators
		Chat chat (0, 550, 250, 300, _frame)
		//operators_list.height =:> chat.y
	}

}