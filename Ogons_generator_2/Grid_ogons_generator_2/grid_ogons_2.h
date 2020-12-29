                     /**************************************************************************************/
                    /*                          Generating Grid Ogons and Ogons                           */
                   /**************************************************************************************/
                  /* Copyright (c) 2015-2017 Faculty of Sciences of the University of Porto (Portugal). */
                 /*  All rights reserved.                                                              */
                /*                                                                                    */
               /* This file is part of a project associated with CGAL (www.cgal.org).                */
              /*  You can redistribute it and/or modify it under the terms of the GNU               */
             /*   General Public License as published by the Free Software Foundation,             */
            /*    either version 3 of the License, or (at your option) any later version.         */
           /*                                                                                    */
          /* Licenses holding a valid commercial license may use this file in accordance        */
         /*  with the commercial license agreement provided with the software.                 */
        /*                                                                                    */
       /* This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE            */
      /*  WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.         */
     /*                                                                                    */
    /* Authors: Ana Paula Tomás <apt@dcc.fc.up.pt>, DCC/FCUP                              */
   /*           Catarina Lobo Ferreira <up201105249@fc.up.pt>, DCC/FCUP                  */
  /*                                                                                    */
 /* Last update: 25/03/2017                                                            */
/**************************************************************************************/


#ifndef CGAL_GRID_OGONS_2_H
#define CGAL_GRID_OGONS_2_H

#include "Grid_ogons_generator_2.h"


namespace CGAL {

  /* Generate a grid ogon by Inflate-Paste Algorithm. */
  Grid_ogon_2 Grid_ogons_generator_2::generate_grid_ogon() {
    
    init_grid_ogon();
    int n_reflex_vertices = (n_vertices-4)/2;

    while(n_reflex_vertices-- > 0) {
            
      int v = convex_vertices[rand()%(current_dim+2)];
      int v_H_adjacent = get_adjacent(v,CGAL_H);  

      FSN_info* fsn_info = get_FSN(v,v_H_adjacent);

      int* cell_coordinates = get_random_cell_in_FSN(fsn_info);

      bool cell_is_in_eH;
      inflate_paste(v,v_H_adjacent,cell_coordinates,cell_is_in_eH);

      /* Update convex_vertices set. */
      int c = get_adjacent(v,CGAL_H);
      vertex_info[c].convex_id = n_convex_vertices;
      convex_vertices[n_convex_vertices] = c;
      n_convex_vertices++; 

      if(!cell_is_in_eH) { //cell is not inside of eH
        int c_V_adjacent = get_adjacent(c,CGAL_V);
        vertex_info[c_V_adjacent].convex_id = vertex_info[v_H_adjacent].convex_id;
        convex_vertices[vertex_info[v_H_adjacent].convex_id] = c_V_adjacent;
        vertex_info[v_H_adjacent].convex_id = -1;
      }

      delete [] cell_coordinates;
    }

    return get_grid_ogon();
  }

  /* Create the unit square grid ogon. */
  void Grid_ogons_generator_2::init_grid_ogon() {

    current_dim = 2;

    /* Init the first four vertices. */
    grid[CGAL_H][1] = 1;
    grid[CGAL_V][1] = 1;
    grid[CGAL_H][2] = 2;
    grid[CGAL_V][2] = 2;

    line_info[CGAL_H][1].coordinate = 1;
    line_info[CGAL_V][1].coordinate = 1;
    line_info[CGAL_H][2].coordinate = 2;
    line_info[CGAL_V][2].coordinate = 2;

    vertex_info[0] = Vertex_2(1,1,3,3,0);
    vertex_info[1] = Vertex_2(2,1,1);
    vertex_info[2] = Vertex_2(2,2,2);
    vertex_info[3] = Vertex_2(1,2,0,0,3);
    add_vertices(0,1,2);
    
    line_info[CGAL_H][1].vertex_id = 0;
    line_info[CGAL_V][1].vertex_id = 0;
    line_info[CGAL_H][2].vertex_id = 2;
    line_info[CGAL_V][2].vertex_id = 2;

    /* Init the set of convex vertices. */
    n_convex_vertices = 4;
    
    convex_vertices[0] = 0;
    convex_vertices[1] = 1;
    convex_vertices[2] = 2;
    convex_vertices[3] = 3;

  }

  /* Insert fst_new vertex and snd_new_vertex id vertices into the vertex_info circular list, next to id vertex start_vertex. vertex_info is in CCW order. */
  void Grid_ogons_generator_2::add_vertices(int start_vertex, int fst_new_vertex, int snd_new_vertex) {

    vertex_info[vertex_info[start_vertex].next].prev = snd_new_vertex;

    vertex_info[snd_new_vertex].next = vertex_info[start_vertex].next;
    vertex_info[snd_new_vertex].prev = fst_new_vertex;

    vertex_info[fst_new_vertex].next = snd_new_vertex;
    vertex_info[fst_new_vertex].prev = start_vertex;

    vertex_info[start_vertex].next = fst_new_vertex;
  }

  /* Return the id of the vertex that is horizontally (vertically) adjacent to vertex with id v, if axis = CGAL_H (axis = CGAL_V). */
  int Grid_ogons_generator_2::get_adjacent(int v, int axis) {

    if(vertex_info[vertex_info[v].next].line_id[axis]==vertex_info[v].line_id[axis])
      return vertex_info[v].next;

    return vertex_info[v].prev;
  }

  /* Return the axis-coordinate of the vertex with id v. */
  int Grid_ogons_generator_2::get_coordinate(int v, int axis) {

    return line_info[axis][vertex_info[v].line_id[axis]].coordinate;
  }

  /* Verify if the horizontal edge with y-coordinate edge_coordinate contains the x-coordinate of the vertex with id v, if axis = CGAL_H.
     Verify if the vertical edge with x-coordinate edge_coordinate contains the y-coordinate of the vertex with id v, if axis = CGAL_V. */
  bool Grid_ogons_generator_2::is_vertex_in_edge(int v, int edge_coordinate, int axis) {

    int the_other_axis = (axis+1)%2;

    int fst_vertex_coordinate = get_coordinate(line_info[axis][grid[axis][edge_coordinate]].vertex_id,the_other_axis);
    int snd_vertex_coordinate = get_coordinate(get_adjacent(line_info[axis][grid[axis][edge_coordinate]].vertex_id,axis),the_other_axis);

    return std::min(fst_vertex_coordinate,snd_vertex_coordinate)<=get_coordinate(v,the_other_axis) &&
            get_coordinate(v,the_other_axis)<=std::max(fst_vertex_coordinate,snd_vertex_coordinate);
  }
 
  /* Verify if horizontal edges with y-coordinates fst_edge_coordinate and snd_edge_coordinate contain some common x-coordinate, if axis = CGAL_H.
     Verify if vertical edges with x-coordinates fst_edge_coordinate and snd_edge_coordinate contain some common y-coordinate, if axis = CGAL_V. */
  bool Grid_ogons_generator_2::do_edges_intersect(int fst_edge_coordinate, int snd_edge_coordinate, int axis) {

    int fst_edge_fst_vertex = line_info[axis][grid[axis][fst_edge_coordinate]].vertex_id;
    int fst_edge_snd_vertex = get_adjacent(line_info[axis][grid[axis][fst_edge_coordinate]].vertex_id,axis);

    int snd_edge_fst_vertex = line_info[axis][grid[axis][snd_edge_coordinate]].vertex_id;
    int snd_edge_snd_vertex = get_adjacent(line_info[axis][grid[axis][snd_edge_coordinate]].vertex_id,axis);

    return is_vertex_in_edge(fst_edge_fst_vertex,snd_edge_coordinate,axis) || is_vertex_in_edge(fst_edge_snd_vertex,snd_edge_coordinate,axis) ||
            is_vertex_in_edge(snd_edge_fst_vertex,fst_edge_coordinate,axis) || is_vertex_in_edge(snd_edge_snd_vertex,fst_edge_coordinate,axis);
  }

  /* Check if the coordinate is in the boundary of the grid. */
  bool Grid_ogons_generator_2::is_coordinate_in_grid_boundary(int coordinate) {

    if(coordinate==0)
      return true;
    if(coordinate==current_dim+1)
      return true;
    return false;
  }

  /* Find the x-coordinate value that cuts horizontaly the rectangular visibility of vertex with id v. */
  int Grid_ogons_generator_2::get_FSN_x_limit(int v, int delta_h, int delta_v) {
    
    int current_x;
    for(current_x=get_coordinate(v,CGAL_X)+delta_v; !is_coordinate_in_grid_boundary(current_x); current_x+=delta_v) {
      int less_y = std::min(get_coordinate(line_info[CGAL_V][grid[CGAL_V][current_x]].vertex_id,CGAL_Y),
                        get_coordinate(get_adjacent(line_info[CGAL_V][grid[CGAL_V][current_x]].vertex_id,CGAL_V),CGAL_Y));
      int greater_y = std::max(get_coordinate(line_info[CGAL_V][grid[CGAL_V][current_x]].vertex_id,CGAL_Y),
                        get_coordinate(get_adjacent(line_info[CGAL_V][grid[CGAL_V][current_x]].vertex_id,CGAL_V),CGAL_Y));

      if(delta_h==-1 && is_vertex_in_edge(v,current_x,CGAL_V) && less_y<get_coordinate(v,CGAL_Y))
        return current_x;
      if(delta_h==1 && is_vertex_in_edge(v,current_x,CGAL_V) && greater_y>get_coordinate(v,CGAL_Y))
        return current_x;
    }
    
    return current_x;
  }

  /* Determine a rectangular partition of the FSN of the vertex with id v. */
  FSN_info* Grid_ogons_generator_2::get_rectangular_visibility(int v, int x_limit, int delta_h, int delta_v) {

    FSN_info* fsn_info = new FSN_info(current_dim+1,delta_h,delta_v);
    
    FSN rectangle;
    rectangle.y_start = get_coordinate(v,CGAL_Y);
    rectangle.x_interval[CGAL_MIN] = std::min(get_coordinate(v,CGAL_X),x_limit);
    rectangle.x_interval[CGAL_MAX] = std::max(get_coordinate(v,CGAL_X),x_limit);

    int current_y;
    for(current_y=rectangle.y_start+delta_h; !is_coordinate_in_grid_boundary(current_y) && !is_vertex_in_edge(v,current_y,CGAL_H); current_y+=delta_h) {
      int less_x = std::min(get_coordinate(line_info[CGAL_H][grid[CGAL_H][current_y]].vertex_id,CGAL_X),
                        get_coordinate(get_adjacent(line_info[CGAL_H][grid[CGAL_H][current_y]].vertex_id,CGAL_H),CGAL_X));
      int greater_x = std::max(get_coordinate(line_info[CGAL_H][grid[CGAL_H][current_y]].vertex_id,CGAL_X),
                          get_coordinate(get_adjacent(line_info[CGAL_H][grid[CGAL_H][current_y]].vertex_id,CGAL_H),CGAL_X));

      if((less_x>rectangle.x_interval[CGAL_MIN] && less_x<rectangle.x_interval[CGAL_MAX]) ||
          (greater_x>rectangle.x_interval[CGAL_MIN] && greater_x<rectangle.x_interval[CGAL_MAX])) {
        
        rectangle.y_finish = current_y;
        fsn_info->add_rectangle(rectangle);

        rectangle.y_start = current_y;
        if(delta_v==-1)
          rectangle.x_interval[CGAL_MIN] = greater_x;
        else //delta_v==1
          rectangle.x_interval[CGAL_MAX] = less_x;
      }
    }
    rectangle.y_finish = current_y;
    fsn_info->add_rectangle(rectangle);

    return fsn_info;
  }

  /* Return the FSN of the vertex with id v. */
  FSN_info* Grid_ogons_generator_2::get_FSN(int v, int v_H_adjacent) {
    
    int v_V_adjacent = get_adjacent(v,CGAL_V);

    int delta_h = (get_coordinate(v,CGAL_Y)-get_coordinate(v_V_adjacent,CGAL_Y))/
                  abs(get_coordinate(v,CGAL_Y)-get_coordinate(v_V_adjacent,CGAL_Y));
    int delta_v = (get_coordinate(v_H_adjacent,CGAL_X)-get_coordinate(v,CGAL_X))/
                  abs(get_coordinate(v_H_adjacent,CGAL_X)-get_coordinate(v,CGAL_X));

    return get_rectangular_visibility(v,get_FSN_x_limit(v,delta_h,delta_v),delta_h,delta_v);
  }

  /* Pick randomly a cell in the FSN fsn_info and determine the coordinates of the northwest corner of that cell. */
  int* Grid_ogons_generator_2::get_random_cell_in_FSN(FSN_info* fsn_info) {
    
    int accumulated_area[fsn_info->n_rectangles+1];
    accumulated_area[0] = 0;
    for(int i=0; i<fsn_info->n_rectangles; i++) {
      accumulated_area[i+1] = accumulated_area[i] + 
                            (fsn_info->rectangles[i].x_interval[CGAL_MAX]-fsn_info->rectangles[i].x_interval[CGAL_MIN])*
                             abs(fsn_info->rectangles[i].y_finish-fsn_info->rectangles[i].y_start);
    }

    int cell = rand()%accumulated_area[fsn_info->n_rectangles]+1;

    int i;
    for(i=0; i<fsn_info->n_rectangles && cell>accumulated_area[i+1]; i++);
    /*if(i==fsn_info->n_rectangles)
      cout << "Error occurs in \"Random_n_ogon_2::get_random_cell_in_FSN\" function" << endl;*/

    int resized_cell = cell-accumulated_area[i];
    int interval_size = fsn_info->rectangles[i].x_interval[CGAL_MAX]-fsn_info->rectangles[i].x_interval[CGAL_MIN]; 

    int* cell_coordinates = new int[2];
    cell_coordinates[CGAL_X] = fsn_info->rectangles[i].x_interval[(fsn_info->delta_v+2)%3]+fsn_info->delta_v*((resized_cell-1)%interval_size)-(fsn_info->delta_v+2)%3;    
    cell_coordinates[CGAL_Y] = fsn_info->rectangles[i].y_start+fsn_info->delta_h*((resized_cell-1)/interval_size)-(fsn_info->delta_h+2)%3;

    return cell_coordinates;
  }

  /* Inflate procedure. */
  void Grid_ogons_generator_2::inflate(int* cell_coordinates, int axis) {

    for(int i=current_dim; i>cell_coordinates[axis]; i--) {
      grid[axis][i+1] = grid[axis][i];
      line_info[axis][grid[axis][i]].coordinate = i+1;
    }
    //grid[axis][cell_coordinates[axis]+1] is now free!
    grid[axis][cell_coordinates[axis]+1] = current_dim+1;
    line_info[axis][current_dim+1].coordinate = cell_coordinates[axis]+1;

    cell_coordinates[axis]++;
  }

  /* Paste procedure. */
  void Grid_ogons_generator_2::paste(int v, int v_H_adjacent, int* cell_coordinates, bool &cell_is_in_eH) {

    int c = 2*current_dim;
    int c_V_adjacent = 2*current_dim+1;

    line_info[CGAL_H][grid[CGAL_H][cell_coordinates[CGAL_Y]]].vertex_id = c;
    line_info[CGAL_V][grid[CGAL_V][cell_coordinates[CGAL_X]]].vertex_id = c;
    vertex_info[c] = Vertex_2(grid[CGAL_H][cell_coordinates[CGAL_Y]],grid[CGAL_V][cell_coordinates[CGAL_X]],-1);

    vertex_info[c_V_adjacent] = Vertex_2(grid[CGAL_H][get_coordinate(v,CGAL_Y)],grid[CGAL_V][cell_coordinates[CGAL_X]],-1);

    cell_is_in_eH = is_vertex_in_edge(c_V_adjacent,get_coordinate(v_H_adjacent,CGAL_Y),CGAL_H);

    line_info[CGAL_H][grid[CGAL_H][get_coordinate(v,CGAL_Y)]].vertex_id = c_V_adjacent;
    vertex_info[v].line_id[CGAL_H] = grid[CGAL_H][cell_coordinates[CGAL_Y]];

    if(vertex_info[v].next==v_H_adjacent)
      add_vertices(v,c,c_V_adjacent);
    else //vertex_info[v_H_adjacent].next==v
      add_vertices(v_H_adjacent,c_V_adjacent,c);
  }

  /* Inflate-Paste */
  void Grid_ogons_generator_2::inflate_paste(int v, int v_H_adjacent, int* cell_coordinates, bool &cell_is_in_eH) {
    
    inflate(cell_coordinates,CGAL_V);
    inflate(cell_coordinates,CGAL_H); 

    paste(v,v_H_adjacent,cell_coordinates,cell_is_in_eH);

    current_dim++;
  }

  /* Return the latest grid ogon generated. */
  std::vector< std::pair<int,int> > Grid_ogons_generator_2::get_grid_ogon() {

    std::vector< std::pair<int,int> > grid_ogon;

    int first_vertex = line_info[CGAL_H][1].vertex_id;
    int i = first_vertex;
    do{
      grid_ogon.push_back(std::pair<int,int>(get_coordinate(i,CGAL_X)-1, get_coordinate(i,CGAL_Y)-1));
      i = vertex_info[i].prev;
    } while(i!=first_vertex);
    
    return grid_ogon;
  }
}

#endif // CGAL_GRID_OGONS_2_H //