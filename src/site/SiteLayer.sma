use core
use gui
use base
use display

import Lima
import ExclusionArea

_define_
SiteLayer (Process _map, Process _context, Process _model_manager)
{
	//map aka _map
	//context aka _context
	//model_manager aka _model_manager

	List exclusion_areas

	List limas

	if (_model_manager.IS_DEBUG)
	{
		addChildrenTo limas {
			Lima debug_lima (_map, null)
		}
	}
}