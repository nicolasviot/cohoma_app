use core
use gui
use base

import widgets.Button
import gui.widgets.StandAlonePushButton
import itinerary.ItineraryPanel
import ClockComponent
import Console

_define_
RightPanel (Process _context, Process _model_manager, Process _frame, Process _ros_node)
{
	//context aka _context

	Translation tr (0, 0)
	_frame.width - _context.RIGHT_PANEL_WIDTH =:> tr.tx

	//FillOpacity _ (0.5)
  	FillColor _ ($_context.DRAK_GRAY)
  	//OutlineColor _ (#FF0000)
  	//OutlineWidth _ (2)
  	Rectangle bg (0, 0, $_context.RIGHT_PANEL_WIDTH, 0, 0, 0)
	_frame.height =:> bg.height

	Spike plan_request
	Spike update_graph
	Spike send_selected_tasks


	// Legend for Nav Graph
	Component nav_graph_legend
	{
		nav_svg = load_from_XML ("res/svg/GraphNav_legend.svg")
		nav << nav_svg.GraphNav

		button aka nav.update_button.update_bg
		button_update_graph aka nav.update_graph_button.update_graph_bg
		FSM button_FSM {
			State idle{
				#666682 =: button.fill.value
				#FFFFFF =: button.stroke.value
			}
			State hover{
				#8080FF =: button.stroke.value
				#666682 =: button.fill.value
			}
			State pressed{
				#1A1AFF =: button.fill.value
				#8080FF =: button.stroke.value
			}
			idle -> hover (button.enter)
			hover -> idle (button.leave)
			idle -> pressed (button.press)
			pressed -> hover (button.release, plan_request)
			hover -> pressed (button.press)
			pressed -> idle (button.leave)
		}

		FSM button_update_graph_FSM {
			State idle{
				#666682 =: button_update_graph.fill.value
				#FFFFFF =: button_update_graph.stroke.value
			}
			State hover{
				#8080FF =: button_update_graph.stroke.value
				#666682 =: button_update_graph.fill.value
			}
			State pressed{
				#1A1AFF =: button_update_graph.fill.value
				#8080FF =: button_update_graph.stroke.value
			}
			idle -> hover (button_update_graph.enter)
			hover -> idle (button_update_graph.leave)
			idle -> pressed (button_update_graph.press)
			pressed -> hover (button_update_graph.release, update_graph)
			hover -> pressed (button_update_graph.press)
			pressed -> idle (button_update_graph.leave)
		}
	}


	Translation _ (0, 125)
	
	// Panel with the 3 strips corresponding to 3 itineraries
	ItineraryPanel itinerary_panel (_context, _model_manager)

	// Start / Stop waiting animation
	plan_request -> itinerary_panel.start_waiting_anim		
	_model_manager.itineraries_updated -> itinerary_panel.stop_waiting_anim


	Translation _ (0, 200)

	Component validate_tasks_button
	{
		validate_tasks_button_svg = load_from_XML ("res/svg/RightPanel_button.svg")
		task_button << validate_tasks_button_svg.button

		button aka task_button.bg
		button_text aka task_button.text.text //un peu bizarre...

		FSM button_FSM {
			State idle{
				#666682 =: button.fill.value
				#FFFFFF =: button.stroke.value
			}
			State hover{
				#8080FF =: button.stroke.value
				#666682 =: button.fill.value
			}
			State pressed{
				#1A1AFF =: button.fill.value
				#8080FF =: button.stroke.value
			}
			idle -> hover (button.enter)
			hover -> idle (button.leave)
			idle -> pressed (button.press)
			pressed -> hover (button.release, send_selected_tasks)
			hover -> pressed (button.press)
			pressed -> idle (button.leave)
		}
	}

	Translation _ (0, 70)
	
	ClockComponent clock (_frame)

	Component ms_per_frame {
		FillColor white (#FFFFFF)
		
		FontSize _ (5, 12) // 5 = pixel
		Text label_mspf (315, -5, "mspf: ")
		
		FontSize _ (5, 14) // 5 = pixel
		Text mspf (350, -5, "0")
		_frame.mspf => mspf.text
	}


	//Translation _ (0, 200)
	//Image icare_logo ("res/icare-logo-2.png", 20, 200, 400, 400)

	Console console(0, 0, _ros_node)

	Component debug {		
		if (_model_manager.IS_DEBUG)
		{
			StandAlonePushButton btn_add_trap1 ("add trap 1", 20, 10)
			btn_add_trap1.click -> _model_manager.add_trap1

			StandAlonePushButton btn_add_trap2 ("add trap 2", 150, 10)
			btn_add_trap2.click -> _model_manager.add_trap2

			StandAlonePushButton btn_set_trap1 ("set trap 1", 20, 40)
			btn_set_trap1.click -> _model_manager.set_trap1

			StandAlonePushButton btn_set_trap2 ("set trap 2", 150, 40)
			btn_set_trap2.click -> _model_manager.set_trap2


			StandAlonePushButton btn_set_shortest ("set shortest", 20, 70)
			btn_set_shortest.click -> _model_manager.set_shortest_itinerary

			StandAlonePushButton btn_set_safest ("set safest", 150, 70)
			btn_set_safest.click -> _model_manager.set_safest_itinerary

			StandAlonePushButton btn_set_tradeoff ("set tradeoff", 280, 70)
			btn_set_tradeoff.click -> _model_manager.set_tradeoff_itinerary
		}
	}

}