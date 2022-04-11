use core
use base
use gui
use display



_define_
AreaTaskDescriptor (Process arg, double x, double y, string text){

	Translation t (x, y)
	FillColor somewhatred(180, 30, 30)
	FillOpacity fo(0.5)


	Rectangle icon (0, 0, 40, 20)


	Text descriptor (45, 15, text)


}