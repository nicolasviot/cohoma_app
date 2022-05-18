use core
use gui
use display
use base

import gui.widgets.StandAlonePushButton

_define_
Console(double _dx, double _dy){


//compte rendu spec :

/*

sur evenement (initiative opérateur)
(divers)


pour les pièges : 
2quipe Icare, nous sommes sur [Position du VAB]
piège identifié ici : 


demande de déplacement du VAB (non)


log à chaque changement de piège


point de situation
position équipe vab
état de tous les pièges (identifié / désactivés)

impondérable subis notes à cotés

listes 




logger activation de lima

*/



/*	Translation t (_dx, _dy)
	
	ite_svg = loadFromXML ("res/svg/CompteRendu.svg")
	ite_gfx << ite_svg.itinerary_panel
*/


  FillColor _(#FAFAFA)
  OutlineColor _ (#FFFFFF)
  Rectangle bg (0, 0, 424, 570)
  FillColor _ (Black)
  
  MultilineEditor ste (5, 0, 0,  0, "", 1)
  2 =: ste.spaces_for_tab
  bg.height =:> ste.height
  bg.width =:> ste.width

  /*StandAlonePushButton debug_ste_input("Clear", 200, 200)
  debug_ste_input.click -> {"Hello World \n"=:ste.string_input}
  */StandAlonePushButton clear("Clear", 200, 500)
  bg.width - 60 =:> clear.x
  bg.height - 30 =:> clear.y
  clear.click -> ste.clear












}