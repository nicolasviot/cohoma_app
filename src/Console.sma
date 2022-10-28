use core
use gui
use display
use base

import gui.widgets.StandAlonePushButton
import ros_node

_action_
save_console (Process c)
%{

    Process *data = (Process*) get_native_user_data(c);
    RosNode *node = dynamic_cast<RosNode*>(data->find_child("node"));
    #ifndef NO_ROS
    node ->save_console(); 
    #endif
    
%}

_define_
Console (double _dx, double _dy, Process _node)
{
  node aka _node
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
    
    svg = loadFromXML ("res/svg/CompteRendu.svg")
    gfx << svg. ...
  */


  FillColor _(#FAFAFA)
  OutlineColor _ (#FFFFFF)
  Rectangle bg (0, 0, 424, 570)
  FillColor _ (Black)
  
  MultilineEditor ste (5, 0, 0,  0, "", 1)
  2 =: ste.spaces_for_tab
  bg.height =:> ste.height
  bg.width =:> ste.width

  /*StandAlonePushButton debug_ste_input("debug", 100, 500)
  debug_ste_input.click -> {"Hello World \n"=:ste.string_input}
  */
  StandAlonePushButton clear("Clear", 200, 500)
  StandAlonePushButton save ("Save", 150, 500)
  bg.width - 60 =:> clear.x
  bg.height - 30 =:> clear.y
  bg.width - 120 =:> save.x
  bg.height - 30 =:> save.y
  clear.click -> ste.clear



  NativeAction save_console_action (save_console, this, 1)
  save.click -> save_console_action

}