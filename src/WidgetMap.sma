use core
use base

import gui.widgets.IWidget
import Map

_define_
WidgetMap (Process f, Process container, int width, int height, double _lat, double _lon, int _zoom) inherits IWidget (container)
{
    Map map (f, 0, 0, width, height, _lat, _lon, _zoom)
    x aka map.x
    y aka map.y

    //this.width =:> map.width
    //this.height =:> map.height
    this.min_height = height
    this.min_width = width
}
