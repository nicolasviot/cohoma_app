use core
use gui
use base

import widgets.Button
import gui.widgets.StandAlonePushButton
import itinerary.ItineraryPanel
import Console

_define_
RightPanel (Process _context, Process _model_manager, Process _frame, Process _ros_node)
{
	//context aka _context

	Spike plan_request
	Spike update_graph
	Spike send_selected_tasks

	Layer layer (0, $_context.TOP_BAR_HEIGHT, 0, 1054) { // Bindings on x and w under

		Translation tr (0, $_context.TOP_BAR_HEIGHT)
		_frame.width - _context.RIGHT_PANEL_WIDTH =:> tr.tx

		FillColor _ ($_context.DARK_GRAY)
		Rectangle bg (0, 0, $_context.RIGHT_PANEL_WIDTH, 0, 0, 0)
		_frame.height =:> bg.height


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

		// Translation _ (0, 200)
		// Image icare_logo ("res/icare-logo-2.png", 20, 200, 400, 400)

		Console console(0, 0, _ros_node)

		Component debug {		
			if (_model_manager.IS_DEBUG)
			{
				StandAlonePushButton btn_set_trap100 ("set trap 100", 20, 10)
				btn_set_trap100.click -> _model_manager.set_trap100

				StandAlonePushButton btn_set_trap101 ("set trap 101", 150, 10)
				btn_set_trap101.click -> _model_manager.set_trap101

				StandAlonePushButton btn_set_trap102 ("set trap 102", 20, 40)
				btn_set_trap102.click -> _model_manager.set_trap102

				StandAlonePushButton btn_set_trap103 ("set trap 103", 150, 40)
				btn_set_trap103.click -> _model_manager.set_trap103


				StandAlonePushButton btn_set_shortest ("set shortest", 20, 80)
				btn_set_shortest.click -> _model_manager.set_shortest_itinerary

				StandAlonePushButton btn_set_safest ("set safest", 150, 80)
				btn_set_safest.click -> _model_manager.set_safest_itinerary

				StandAlonePushButton btn_set_tradeoff ("set tradeoff", 280, 80)
				btn_set_tradeoff.click -> _model_manager.set_tradeoff_itinerary


				StandAlonePushButton btn_add_task_edge1 ("add task edge 1", 20, 120)
				btn_add_task_edge1.click -> _model_manager.add_task_edge1

				StandAlonePushButton btn_add_task_edge2 ("add task edge 2", 150, 120)
				btn_add_task_edge2.click -> _model_manager.add_task_edge2
			}
		}
	}
	_frame.width - _context.RIGHT_PANEL_WIDTH =:> layer.x
	//_context.RIGHT_PANEL_WIDTH =:> layer.w
	_context.RIGHT_PANEL_WIDTH =: layer.w
}