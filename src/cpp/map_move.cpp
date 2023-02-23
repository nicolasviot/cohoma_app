#include <cmath>

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"

#include "cpp/coords-utils.h"
#include "cpp/map_move.h"
#include "core/core.h"

  void fill4tiles (djnn::CoreProcess* tiles, int nbRows, int nbCols, int row, int col, int x, int y, int z) {
    auto * tile = tiles->find_child (row)->find_child(col);

    if (((djnn::AbstractProperty*)tile->find_child ("updated"))->get_double_value() == 1)
      return;

    ((djnn::AbstractProperty*)tile->find_child( "Z"))->set_value (z, true);
    ((djnn::AbstractProperty*)tile->find_child ("updated"))->set_value(1, true);
    ((djnn::AbstractProperty*)tile->find_child( "X"))->set_value (x, true);
    ((djnn::AbstractProperty*)tile->find_child( "Y"))->set_value (y, true);

    //up
    if (row > 1) {
      fill4tiles (tiles, nbRows, nbCols, row-1, col, x, y-1, z);
    }
    //down
    if (row < nbRows) {
      fill4tiles (tiles, nbRows, nbCols, row+1, col, x, y+1, z);
    }
    //right
    if (col < nbCols) {
      fill4tiles (tiles, nbRows, nbCols, row, col+1, x+1, y, z);
    }
    //left
    if (col > 1) {
      fill4tiles (tiles, nbRows, nbCols, row, col-1, x-1, y, z);
    }
  }

  void
  fn_zoom_in (djnn::CoreProcess *src)
  {
    djnn::CoreProcess *data = (djnn::CoreProcess*) djnn::get_native_user_data (src);

    int nbCols = djnn::getInt (data->find_child ("nbCols"));

    int nbRows = djnn::getInt (data->find_child ("nbRows"));
    int row = djnn::getInt (data->find_child ("pointer_row"));
    int col = djnn::getInt (data->find_child ("pointer_col"));
    int z = djnn::getInt (data->find_child ("zoomLevel")) + 1;
    double lon = djnn::getDouble (data->find_child ("pointer_lon"));
    double lat = djnn::getDouble (data->find_child ("pointer_lat"));

    int tx =  lon2tilex (lon, z); // x de la nouvelle tuile couvrant le point de référence
    int ty =  lat2tiley (lat, z); // y de la nouvelle tuile couvrant le point de référence
    


    djnn::List *tiles = (djnn::List*) data->find_optional_child ("tile_layer/tiles");
    djnn::CoreProcess* tile = tiles->find_child (std::to_string(row))->find_child(std::to_string (col));
    if (tiles == nullptr) {
      djnn::release_exclusive_access(DBG_REL);
      return;
    }
    for (int n_row = 1; n_row <= nbRows; n_row++) {
      for (int n_col = 1; n_col <= nbCols; n_col++) {
        djnn::CoreProcess* tile = ((djnn::List*)tiles->find_child (n_row))->find_child(n_col);
        ((djnn::AbstractProperty*)tile->find_child( "updated"))->set_value (0, true);
      }
    }
    fill4tiles (tiles, nbRows, nbCols, row + 1, col + 1, tx, ty, z);
  }

void
fn_zoom_out (djnn::CoreProcess *src)
{

  djnn::Process *data = (djnn::CoreProcess*) djnn::get_native_user_data (src);

  int nbCols = djnn::getInt (data->find_child ("nbCols"));
  int nbRows = djnn::getInt (data->find_child ("nbRows"));
  int row = djnn::getInt (data->find_child ("pointer_row"));
  int col = djnn::getInt (data->find_child ("pointer_col"));
  int z = djnn::getInt (data->find_child ("zoomLevel")) - 1;
  double lon = djnn::getDouble (data->find_child ("pointer_lon"));
  double lat = djnn::getDouble (data->find_child ("pointer_lat"));
  int tx =  lon2tilex (lon, z); // x de la tuile couvrant le point de référence
  int ty =  lat2tiley (lat, z); // y de la tuile couvrant le point de référence
  djnn::List *tiles = (djnn::List*) data->find_optional_child ("tile_layer/tiles");
  djnn::CoreProcess* tile = tiles->find_child (std::to_string(row))->find_child(std::to_string (col));
  if (tiles == nullptr) {
    djnn::release_exclusive_access(DBG_REL);
    return;
  }
  for (int n_row = 1; n_row <= nbRows; n_row++) {
    for (int n_col = 1; n_col <= nbCols; n_col++) {
      djnn::CoreProcess* tile = ((djnn::List*)tiles->find_child (n_row))->find_child(n_col);
      ((djnn::AbstractProperty*)tile->find_child( "updated"))->set_value (0, true);
    }
  }
  fill4tiles (tiles, nbRows, nbCols, row + 1, col + 1, tx, ty, z);
  }

void 
fn_move_right (djnn::CoreProcess *src)
  {
    djnn::CoreProcess *data = (djnn::CoreProcess*) djnn::get_native_user_data (src);

    djnn::CoreProcess *tiles = data->find_optional_child ("tile_layer/tiles");
    if (tiles == nullptr) {
      djnn::release_exclusive_access(DBG_REL);
      return;
    }
    int nbCol = getInt (data->find_child ("nbCols"));
    int nbRow = getInt (data->find_child ("nbRows"));
    int off_x = abs(getInt (data->find_child ("offset_x")));

    for (int nb = 0; nb < off_x; nb++) {
      djnn::List* first_row = (djnn::List*)tiles->find_child ("1");
      djnn::List* last_row = (djnn::List*)tiles->find_child (nbRow);
      djnn::CoreProcess* first_tile = first_row->find_child ("1");
      djnn::CoreProcess* last_tile = last_row->find_child (std::to_string (nbCol));
    
      int X1 = djnn::getInt (first_tile->find_child ("X"));
      double x1 = ((djnn::AbstractProperty*)first_tile->find_child( "img/x"))->get_double_value ();
      for (int i = 1; i <= nbRow; i++) {
        first_row = (djnn::List*)tiles->find_child (std::to_string (i));
        djnn::CoreProcess *tile = first_row->find_child(std::to_string(nbCol));
        djnn::CoreProcess *img_X = tile->find_child("X");
        djnn::setInt (img_X, X1 - 1);
        first_row->remove_child (tile);
        first_row->insert (tile, "<");
        djnn::AbstractProperty *img_x = (djnn::AbstractProperty*)tile->find_child( "img/x");
        img_x->set_value (x1 - 256, true);  
      }
    }
  }

  void
  fn_move_left (djnn::CoreProcess *src)
  {
    djnn::CoreProcess *data = (djnn::CoreProcess*) djnn::get_native_user_data (src);

    djnn::CoreProcess *tiles = data->find_optional_child ("tile_layer/tiles");
    if (tiles == nullptr) {
      djnn::release_exclusive_access(DBG_REL);
      return;
    }
    int nbCol = djnn::getInt (data->find_child ("nbCols"));
    int nbRow = djnn::getInt (data->find_child ("nbRows"));
    int off_x = abs(getInt (data->find_child ("offset_x")));

    for (int nb = 0; nb < off_x; nb++) {
      djnn::List* first_row = (djnn::List*)tiles->find_child ("1");
      djnn::List* last_row = (djnn::List*)tiles->find_child (nbRow);
      djnn::CoreProcess* first_tile = first_row->find_child ("1");
      djnn::CoreProcess* last_tile = last_row->find_child (std::to_string (nbCol));
      int X_last = djnn::getInt (last_tile->find_child ("X"));
      double x_last = ((djnn::AbstractProperty*)last_tile->find_child( "img/x"))->get_double_value ();
      for (int i = 1; i <= nbRow;i++) {
        first_row = (djnn::List*)tiles->find_child (std::to_string (i));
        djnn::CoreProcess *tile = first_row->find_child(std::to_string(1));
        djnn::CoreProcess *img_X = tile->find_child("X");
        djnn::setInt (img_X, X_last + 1);
        first_row->remove_child (tile);
        first_row->insert (tile, ">");
        djnn::AbstractProperty *img_x = (djnn::AbstractProperty*)tile->find_child( "img/x");
        img_x->set_value (x_last + 256, true);
      }
    }
  }

  void
  fn_move_down (djnn::CoreProcess *src)
  {
    djnn::CoreProcess *data = (djnn::CoreProcess*) get_native_user_data (src);
    djnn::CoreProcess *tiles = data->find_optional_child ("tile_layer/tiles");
    if (tiles == nullptr) {
      djnn::release_exclusive_access(DBG_REL);
      return;
    }
    int nbCol = getInt (data->find_child ("nbCols"));
    int nbRow = getInt (data->find_child ("nbRows"));
    int off_y = abs(getInt (data->find_child ("offset_y")));

    for (int nb = 0; nb < off_y; nb++) {
      djnn::List* first_row = (djnn::List*)tiles->find_child ("1");
      djnn::List* last_row = (djnn::List*)tiles->find_child (nbRow);
      djnn::CoreProcess* first_tile = first_row->find_child ("1");
      djnn::CoreProcess* last_tile = last_row->find_child (std::to_string (nbCol));

      int Y_first = getInt(first_tile->find_child("Y"));
      double y1 = ((djnn::AbstractProperty*)first_tile->find_child( "img/y"))->get_double_value ();
      last_row = (djnn::List*)tiles->find_child (nbRow);
      for (int j = 1; j <= nbCol; j++) {
        djnn::CoreProcess *tile = last_row->find_child (std::to_string(j));
        djnn::CoreProcess *tile_Y = tile->find_child("Y");
        djnn::setInt (tile_Y, Y_first - 1);
        djnn::AbstractProperty *img_y = (djnn::AbstractProperty*)tile->find_child( "img/y");
        img_y->set_value (y1 - 256, true);
      }
      tiles->remove_child (last_row);
      ((djnn::List*)tiles)->insert (last_row, "<");
    }
  }

  void
  fn_move_up (djnn::CoreProcess *src)
  {
    djnn::CoreProcess *data = (djnn::CoreProcess*) djnn::get_native_user_data (src);
  
    djnn::CoreProcess *tiles = data->find_optional_child ("tile_layer/tiles");
    if (tiles == nullptr) {
      djnn::release_exclusive_access(DBG_REL);
      return;
    }
    int nbCol = djnn::getInt (data->find_child ("nbCols"));
    int nbRow = djnn::getInt (data->find_child ("nbRows"));
    int off_y = abs(getInt (data->find_child ("offset_y")));
  
    for (int nb = 0; nb < off_y; nb++) {
      djnn::List* first_row = (djnn::List*)tiles->find_child ("1");
      djnn::List* last_row = (djnn::List*)tiles->find_child (nbRow);
      djnn::CoreProcess* first_tile = first_row->find_child ("1");
      djnn::CoreProcess* last_tile = last_row->find_child (std::to_string (nbCol));
      int Y_last = djnn::getInt (last_tile->find_child ("Y"));
      double y_last = ((djnn::AbstractProperty*)last_tile->find_child( "img/y"))->get_double_value ();
      first_row = (djnn::List*)tiles->find_child ("1");
      for (int j = 1; j <= nbCol; j++) {
        djnn::CoreProcess *tile = first_row->find_child (std::to_string(j));
        djnn::CoreProcess *tile_Y = tile->find_child("Y");
        djnn::setInt (tile_Y, Y_last + 1);
        djnn::AbstractProperty *img_y = (djnn::AbstractProperty*)tile->find_child( "img/y");
        img_y->set_value (y_last + 256, true);
      }
      tiles->remove_child (first_row);
      ((djnn::List*)tiles)->insert (first_row, ">");
    }
  }
