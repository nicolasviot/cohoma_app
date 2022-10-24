use core
use gui
use base

import widgets.Button
import itinerary.ItineraryPannel
import ClockComponent
import Console

_define_
RightPannel (Process _context, Process _model_manager, Process _frame, Process _ros_node, Process _itineraries)
{
	Translation tr (0, 0)
	_frame.width - _context.RIGHT_PANEL_WIDTH =:> tr.tx

	Spike plan_request 
	Spike validate_plan  
	Spike update_graph
	Spike test_multiple_itineraries_spike
	Spike test_allocation_spike
	Spike test_lima_spike
	Spike send_selected_tasks
	Spike test_visibility_map

	// Legend for Nav Graph
	Component nav_graph_legend
	{
		nav_svg = loadFromXML ("res/svg/GraphNav_legend.svg")
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


	Translation _(0, 125)
	
	//ItineraryPannel itineraryPannel(0, 0, root.l.map.layers.itineraries.id)
	ItineraryPannel itineraryPannel(0, 0, _itineraries.id)

	plan_request -> itineraryPannel.startWaitingAnim
	//TODO add a stop for the update animation when receiving data from ros 
	// trigger   itineraryPannel.stopWaitingAnim
	//itineraryPannel.first.sw.unselected.bg.press ->  itineraryPannel.stopWaitingAnim

	Translation _ (0, 200)
	Component validate_tasks_button
	{
		validate_tasks_button_svg = loadFromXML ("res/svg/RightPanel_button.svg")
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
	ClockComponent clock(0, -25, _frame)


	//Translation _ (0, 200)
	//Image icare_logo ("res/icare-logo-2.png", 20, 200, 400, 400)

	Console console(0, 0, _ros_node)

	/*Component IObuttons{
		Button send_plan_req (frame, " request plan ", 20, 140)
		send_plan_req.click -> plan_request
		
		Button valid_plan (frame, " validate plan ", 20, 200)
		valid_plan.click -> validate_plan

		Button update_graph_but (frame, "send graph", 20, 260)
		update_graph_but.click -> update_graph
	
		Button test_multiple_itineraries_button (frame, "test multiple itineraries", 20, 320)
		test_multiple_itineraries_button.click -> test_multiple_itineraries_spike
	
		Button test_allocation_button(frame, "test allocation", 20, 380)
		test_allocation_button.click -> test_allocation_spike
		
		Button test_lima_button(frame, "test lima", 20, 440)
		test_lima_button.click -> test_lima_spike

		Button test_visibility_map_button(frame, "test_visibility_map", 20, 500)
		test_visibility_map_button.click->test_visibility_map
	}*/

}