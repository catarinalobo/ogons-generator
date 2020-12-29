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


#ifndef CGAL_GRID_OGONS_GENERATOR_2_H
#define CGAL_GRID_OGONS_GENERATOR_2_H

#include "grid_ogons_generator_macros.h"
#include "grid_ogons_generator_aux_ds.h"

#include "Grid_ogon_2.h"

#include <stdlib.h>
#include <math.h>


namespace CGAL {
  
  /* Generation of grid ogons and some subfamilies, given a fixed number of vertices. */
  class Grid_ogons_generator_2 {         

  public:
    Grid_ogons_generator_2(int n_vertices);
    ~Grid_ogons_generator_2();
		
    Grid_ogon_2 generate_grid_ogon();                  //O(n_vertices^2)
    Grid_ogon_2 generate_row_convex_grid_ogon();       //O(n_vertices)
    Grid_ogon_2 generate_column_convex_grid_ogon();    //O(n_vertices)
    Grid_ogon_2 generate_convex_grid_ogon();           //O(n_vertices)
    Grid_ogon_2 generate_path_grid_ogon();             //O(n_vertices^2)
    Grid_ogon_2 generate_spiral_grid_ogon();           //O(n_vertices^2)
  
  protected:
    int n_vertices;               //n_vertices is the number of vertices
    int current_dim;              //current_dim is equal to n_vertices/2 in the final polygon
    
    int** grid;                   //grid[CGAL_H][i]: id of the horizontal line with y-coordinate equals to i
                                  //grid[CGAL_V][i]: id of the vertical line with x-coordinate equals to i

    Line_info** line_info;        //line_info[CGAL_H][i].coordinate: y-coordinate of the horizontal line with id equals to i     
                                  //line_info[CGAL_V][i].coordinate: x-coordinate of the vertical line with id equals to i
                                  //line_info[CGAL_H][i].vertex_id: one vertex of the horizontal line with id equals to i     
                                  //line_info[CGAL_V][i].vertex_id: one vertex of the vertical line with id equals to i

    Vertex_2* vertex_info;        //vertex_info[i]: vertex of the polygon with id equals to i

    int* convex_vertices;         //set of convex vertices that satisfy certain properties (depending on the grid ogon subclass)
    int n_convex_vertices;        //number of elements in convex_vertices


    int get_adjacent(int v, int axis);
    int get_coordinate(int v, int axis);
    bool is_coordinate_in_grid_boundary(int coordinate);
  
  private:  
    void init_grid_ogon();
    void add_vertices(int start_vertex, int fst_new_vertex, int snd_new_vertex);
    bool is_vertex_in_edge(int v, int edge_coordinate, int axis);
    bool do_edges_intersect(int fst_edge_coordinate, int snd_edge_coordinate, int axis);
  
    int get_FSN_x_limit(int v, int delta_h, int delta_v);
    FSN_info* get_rectangular_visibility(int v, int x_limit, int delta_h, int delta_v);
    FSN_info* get_FSN(int v, int v_H_adjacent);
    int* get_random_cell_in_FSN(FSN_info* fsn_info);

    void inflate(int* cell_coordinates, int axis);
    void paste(int v, int v_H_adjacent, int* cell_coordinates, bool &cell_is_in_eH);
    void inflate_paste(int v, int v_H_adjacent, int* cell_coordinates, bool &cell_is_in_eH);

    FSN_info* get_FSN_row_convex(int v, int v_H_adjacent);
    int* get_random_cell_in_FSN_row_convex(FSN_info* fsn_info);
    
    FSN_info* get_FSN_convex(int v, int v_H_adjacent);

    FSN_info* get_FSN_base_case_path(int v, int v_H_adjacent);
    FSN_info* get_FSN_path(int v, int v_H_adjacent, int* convex_vertex_type_limit);
    FSN_info* refine_FSN_path(int v, FSN_info* fsn_info);

    void add_convex_vertex_path(int vertex, int id, int type_limit, int* convex_vertex_type_limit);
    void init_types_path(int v, int v_H_adjacent, int c, int c_V_adjacent, bool cell_is_in_eH, int* convex_vertex_type_limit);
    void update_types_path(int v, int v_H_adjacent, int c, int c_V_adjacent, int* convex_vertex_type_limit);

    void init_types_spiral(int v, int v_H_adjacent, int c, int c_V_adjacent, bool cell_is_in_eH, int* convex_vertex_type_limit);
    void update_types_spiral(int v, int v_H_adjacent, int c_V_adjacent, int* convex_vertex_type_limit);

    std::vector< std::pair<int,int> > get_grid_ogon();
  };

  /* Constructor */
  Grid_ogons_generator_2::Grid_ogons_generator_2(int n_vertices) {
    
    this->n_vertices = n_vertices;
    current_dim = 0;
    
    /* Init grid lines. */
    grid = new int*[2];
    grid[CGAL_H] = new int[n_vertices/2+1];
    grid[CGAL_V] = new int[n_vertices/2+1];

    line_info = new Line_info*[2];
    line_info[CGAL_H] = new Line_info[n_vertices/2+1];
    line_info[CGAL_V] = new Line_info[n_vertices/2+1];

    /* Init vertices. */
    vertex_info = new Vertex_2[n_vertices];

    /* Init convex_vertices set. */
    convex_vertices = new int[n_vertices/2+2];
  }

  /* Destructor */
  Grid_ogons_generator_2::~Grid_ogons_generator_2() {

    delete [] grid[CGAL_H];
    delete [] grid[CGAL_V];
    delete [] grid;
    grid = NULL;

    delete [] line_info[CGAL_H];
    delete [] line_info[CGAL_V];
    delete [] line_info;
    line_info = NULL;

    delete [] vertex_info;
    vertex_info = NULL;

    delete [] convex_vertices;
    convex_vertices = NULL;
  }
}

#endif // CGAL_GRID_OGONS_GENERATOR_2_H //