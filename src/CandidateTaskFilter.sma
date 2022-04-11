use core 
use gui
use display
use base

import AreaTaskDescriptor



_define_
CandidateTaskFilter (Process arg){

Translation _t(20, 20)




Component AreaTaskFilter{

	List tasks{
		AreaTaskDescriptor _ (arg, 20, 0, "Area 1 ")
		AreaTaskDescriptor _ (arg, 20, 40, "Area 2 ")

	}


}

Component SegmentTaskFilter{

	List tasks{
		
	}



}

Component TrapTaskFilter{


	List tasks{
		
	}


}



}