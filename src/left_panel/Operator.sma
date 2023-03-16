use core
use gui
use display

/*_native_code_
%{
   #include <iostream>
%}*/


_define_
//Operator (Process _context, Process _model, int _index, Process _svg)
Operator (Process _context, Process _model)
{
   //context aka _context
   model aka _model

   Double height ($_context.OPERATOR_HEADER_HEIGHT + 10)

   Translation tr (0, 0)
   y aka tr.ty

   NoFill _
   Rectangle bg (0, 0, $_context.LEFT_PANEL_WIDTH, 0, 0, 0)
   height =:> bg.height

   Component header {
      NoOutline _

      FillColor _ ($_model.color)
      Rectangle r1 (0, 0, 147, $_context.OPERATOR_HEADER_HEIGHT, 0, 0)

      FillColor _ (#AAAAAA)
      Rectangle r2 (147, 0, $_context.LEFT_PANEL_WIDTH - 147, $_context.OPERATOR_HEADER_HEIGHT, 0, 0)

      //FontWeight _ (DJN_BOLD)
      //FontWeight _ (DJN_NORMAL)
      FontSize _ (5, 18) // 5 = pixel
      
      FillColor white (#FFFFFF)
      Text txt_title (5, 25, toString(_model.title))

      FillColor white (#000000)
      Text txt_name (152, 25, toString(_model.name))
   }

   print ("New view of Operator (" + _model.uid + ") type: " + _model.code + " (" + _model.type + ") title: " + _model.title + " named " + _model.name + "\n")

   //Translation tr (_index * $_context.STRIP_WIDTH, 0)

   //g << clone (_svg.Strip)

   //heading.output + "°" =:> g.central.data.heading.heading_text.text
   //alt_msl.output + "m" =:> g.central.data.altitude.altitude_text.text 
   //(model.type == "drone") ? "aérien" : "terrestre" =:> g.type.text
   //model.name =:> g.id.text
   //model.status =:> g.left.status.mode.mode_text.text
   //model.color =:> g.strip_color.fill.value

   //g.background.press -> model.start_locate
   //g.background.release -> model.stop_locate

}

