use core
use gui
use base
use gui
  
import Operator

_define_
OperatorsList (Process _context, Process _model_manager)
{
  context aka _context
  model_manager aka _model_manager

  Int DELTA_Y_BETWEEN_OPERATORS (5)
  Double height (0)
  

  //FillColor _ (#FF0000)
  //Rectangle bg_debug (0, 0, 50, 50, 0, 0)
  //height =:> bg_debug.height
  ZOrderedGroup _operator_z_order_group{

    Operator ot (_context, _model_manager.operators.ot)
    
    Operator og1 (_context, _model_manager.operators.og1)
    ot.y + ot.height + $DELTA_Y_BETWEEN_OPERATORS =:> og1.y

    Operator og2 (_context, _model_manager.operators.og2)
    og1.y + og1.height + $DELTA_Y_BETWEEN_OPERATORS =:> og2.y

    Operator og3 (_context, _model_manager.operators.og3)
    og2.y + og2.height + $DELTA_Y_BETWEEN_OPERATORS =:> og3.y

    og3.y + og3.height =:> height

    //todo filter the correct ones
    _context.show_drop_zones_strip =:> og1.accept_drop
    _context.show_drop_zones_strip =:> og2.accept_drop
    _context.show_drop_zones_strip =:> og3.accept_drop
  }


  //formulaire confirmation de changement opérateur
  form_svg = load_from_XML("res/svg/allocation-robot-confirmation.svg")
  Switch allocation_confirmation_form (hidden) {
      Component hidden {}
      Component visible {
        form << form_svg.form
      }
    }
    confirmation_form aka allocation_confirmation_form.visible.form
 
    FSM fsm_form {
      State hidden
      State visible
      hidden -> visible (context.dropped_operator)
      visible -> hidden (confirmation_form.btn_oui.press)
      visible -> hidden (confirmation_form.btn_non.press)
    }
    fsm_form.state => allocation_confirmation_form.state

  //action lors du drop pour afficher la confirmation (ATTENTION PROBLEM SI PAS LA LIGNE AU DESSUS...)
  context.dropped_operator -> na_find_dropped_strip_model:(this) {
    _dropped_strip = getRef(this.context.dragged_strip)
    _target_operator = getRef(this.context.dropped_operator)
    if (&_dropped_strip != null && &_target_operator != null) {
      this.confirmation_form.robot_name.text = toString(_dropped_strip.model.code)
      this.confirmation_form.op_name.text = toString(_target_operator.model.title)
    }
  }

  //action lorsque l'utilisateur choisit oui et confirme la nouvelle affectation.
  confirmation_form.btn_oui.press -> na_transfer_robot : (this){
    _dropped_strip = getRef(this.context.dragged_strip)
    _prev_operator = getRef(_dropped_strip.model.ref_operator)
    _target_operator = getRef(this.context.dropped_operator)

    if(&_dropped_strip != null && &_prev_operator != null && &_target_operator!= null){
      //remove robot model from prev operator robots (processcollector)
      setRef(_prev_operator.robots.rm , _dropped_strip.model)
      
      //change the strip to the new parent
      addChildrenTo _target_operator.strips_container.strips{
            _dropped_strip
      }
        
      //add the robot to the new operator and set ref to operator
      setRef(_target_operator.model.robots.add , _dropped_strip.model)
      setRef(_dropped_strip.model.ref_operator, _target_operator.model)

      //print ("je confirme que le robot " + _dropped_strip.model.code + " est alloué à l'opérateur " + _target_operator.model.title + "\n")

      //SEND MESSAGE WITH NEW ALLOCATION TO ROS TOPIC
      print("TODO: envoyer nouvelle affectation des robots par messages ROS \n" ) 
    }
  }
}