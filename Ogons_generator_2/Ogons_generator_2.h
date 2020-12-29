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


#ifndef CGAL_OGONS_GENERATOR_2_H
#define CGAL_OGONS_GENERATOR_2_H

#include "Grid_ogons_generator_2/grid_ogons_2.h"
#include "Grid_ogons_generator_2/row_convex_grid_ogons_2.h"
#include "Grid_ogons_generator_2/column_convex_grid_ogons_2.h"
#include "Grid_ogons_generator_2/convex_grid_ogons_2.h"
#include "Grid_ogons_generator_2/path_grid_ogons_2.h"
#include "Grid_ogons_generator_2/spiral_grid_ogons_2.h"

#include "Ogon_2.h"


namespace CGAL {
  
  /* Generation of orthogonal polygons (ogons) and some subfamilies, given a fixed number of vertices. */
  class Ogons_generator_2: public Grid_ogons_generator_2 {         

  public:
    Ogons_generator_2(int n_vertices);
    ~Ogons_generator_2();
		   
    Ogon_2 generate_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings);                  //O(n_vertices*(n_vertices+n_stretchings))
    Ogon_2 generate_row_convex_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings);       //O(n_vertices*(n_vertices+n_stretchings))
    Ogon_2 generate_column_convex_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings);    //O(n_vertices*(n_vertices+n_stretchings))
    Ogon_2 generate_convex_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings);           //O(n_vertices*(n_vertices+n_stretchings))
    Ogon_2 generate_path_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings);             //O(n_vertices*(n_vertices+n_stretchings))
    Ogon_2 generate_spiral_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings);           //O(n_vertices*(n_vertices+n_stretchings))

    Ogon_2 generate_ogon_from_grid_ogon(Grid_ogon_2 grid_ogon, int* max_coordinate, float colinearity_probability, int n_stretchings);   //O(n_vertices*(n_vertices+n_stretchings))

  protected:
    int** stretched;                //to randomly modify the edges length of a grid ogon when it is asked for an orthogonal polygon

    void init_default_stretcher();
    
    bool could_vertex_be_in_edge(int v, int edge_coordinate, int axis);
    bool could_edges_intersect(int fst_edge_coordinate, int snd_edge_coordinate, int axis);
    
    bool is_colinearity_possible(int edge_coordinate, int rise, int axis);
    void make_colinear_edges(int edge_coordinate, float colinearity_probability, int axis);

    void stretch_edges(int edge_coordinate, int* max_coordinate, int n_stretchings, int axis);
    void stretch_grid_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings);

    std::vector< std::pair<int,int> > get_ogon();
  };

  /* Constructor */
  Ogons_generator_2::Ogons_generator_2(int n_vertices): Grid_ogons_generator_2(n_vertices) {

    stretched = new int*[2];
    stretched[CGAL_X] = new int[n_vertices/2+1];
    stretched[CGAL_Y] = new int[n_vertices/2+1];
  }

  /* Destructor */
  Ogons_generator_2::~Ogons_generator_2() {

    delete [] stretched[CGAL_X];
    delete [] stretched[CGAL_Y];
    delete [] stretched;

    stretched = NULL;
  }

  /* Generate an ogon by Inflate-Paste and Stretcher Algorithms. */
  Ogon_2 Ogons_generator_2::generate_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings) {

    Grid_ogon_2 grid_ogon = generate_grid_ogon();
    stretch_grid_ogon(max_coordinate, colinearity_probability, n_stretchings);

    return Ogon_2(grid_ogon, get_ogon());
  }

  /* Generate a row-convex ogon by Inflate-Paste and Stretcher Algorithms. */
  Ogon_2 Ogons_generator_2::generate_row_convex_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings) {

    Grid_ogon_2 grid_ogon = generate_row_convex_grid_ogon();
    stretch_grid_ogon(max_coordinate, colinearity_probability, n_stretchings);

   return Ogon_2(grid_ogon, get_ogon()); 
  }

  /* Generate a column-convex ogon by Inflate-Paste and Stretcher Algorithms. */
  Ogon_2 Ogons_generator_2::generate_column_convex_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings) {

    Grid_ogon_2 grid_ogon = generate_column_convex_grid_ogon();
    stretch_grid_ogon(max_coordinate, colinearity_probability, n_stretchings);

   return Ogon_2(grid_ogon, get_ogon()); 
  }

  /* Generate a convex ogon by Inflate-Paste and Stretcher Algorithms. */
  Ogon_2 Ogons_generator_2::generate_convex_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings) {

    Grid_ogon_2 grid_ogon = generate_convex_grid_ogon();
    stretch_grid_ogon(max_coordinate, colinearity_probability, n_stretchings);
    
    return Ogon_2(grid_ogon, get_ogon());
  }

  /* Generate a path ogon by Inflate-Paste and Stretcher Algorithms. */
  Ogon_2 Ogons_generator_2::generate_path_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings) {

    Grid_ogon_2 grid_ogon = generate_path_grid_ogon();
    stretch_grid_ogon(max_coordinate, colinearity_probability, n_stretchings);
  
    return Ogon_2(grid_ogon, get_ogon());
  }

  /* Generate a spiral ogon by Inflate-Paste and Stretcher Algorithms. */
  Ogon_2 Ogons_generator_2::generate_spiral_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings) {

    Grid_ogon_2 grid_ogon = generate_spiral_grid_ogon();
    stretch_grid_ogon(max_coordinate, colinearity_probability, n_stretchings);
  
    return Ogon_2(grid_ogon, get_ogon());
  }

  /* Generate an ogon from a given grid ogon, by Stretcher Algorithm. */
  Ogon_2 Ogons_generator_2::generate_ogon_from_grid_ogon(Grid_ogon_2 grid_ogon, int* max_coordinate, float colinearity_probability, int n_stretchings) {

    std::list< std::pair<int,int> > vertices = grid_ogon.get_list< std::pair<int,int> >();
    int i=0;
    for(std::list< std::pair<int,int> >::iterator it = vertices.begin(); it!=vertices.end(); it++) {
      
      grid[CGAL_H][it->second+1] = it->second+1;
      grid[CGAL_V][it->first+1] = it->first+1;

      line_info[CGAL_H][it->second+1].coordinate = it->second+1;
      line_info[CGAL_V][it->first+1].coordinate = it->first+1;

      line_info[CGAL_H][it->second+1].vertex_id = i;
      line_info[CGAL_V][it->first+1].vertex_id = i;

      vertex_info[i] = Vertex_2(it->second+1, it->first+1, i+1, i-1, -1);
      
      i++;
    }
    vertex_info[vertices.size()-1].prev = 0;
    vertex_info[0].next = vertices.size()-1;

    
    stretch_grid_ogon(max_coordinate, colinearity_probability, n_stretchings);
  
    return Ogon_2(grid_ogon, get_ogon());
  }

  /* Create the default stretcher, i.e., the ogon to be created is initially the latest grid ogon. */
  void Ogons_generator_2::init_default_stretcher() {

    for(int i=1; i<=current_dim; i++) {
      stretched[CGAL_X][i] = i;
      stretched[CGAL_Y][i] = i;
    }
  }

  /* Verify if the horizontal edge with y-coordinate edge_coordinate contains the x-coordinate of the vertex with id v, if axis = CGAL_Y.
     Verify if the vertical edge with x-coordinate edge_coordinate contains the y-coordinate of the vertex with id v, if axis = CGAL_X.
     edge_coordinate is the original coordinate in the grid ogon. */  
  bool Ogons_generator_2::could_vertex_be_in_edge(int v, int edge_coordinate, int axis) {

    int the_other_axis = (axis+1)%2;

    int fst_vertex_coordinate = stretched[the_other_axis][get_coordinate(line_info[axis][grid[axis][edge_coordinate]].vertex_id,the_other_axis)];
    int snd_vertex_coordinate = stretched[the_other_axis][get_coordinate(get_adjacent(line_info[axis][grid[axis][edge_coordinate]].vertex_id,axis),the_other_axis)];

    return std::min(fst_vertex_coordinate,snd_vertex_coordinate)<=stretched[the_other_axis][get_coordinate(v,the_other_axis)] &&
            stretched[the_other_axis][get_coordinate(v,the_other_axis)]<=std::max(fst_vertex_coordinate,snd_vertex_coordinate);
  }

  /* Verify if horizontal edges with y-coordinates fst_edge_coordinate and snd_edge_coordinate contain some common x-coordinate, if axis = CGAL_Y.
     Verify if vertical edges with x-coordinates fst_edge_coordinate and snd_edge_coordinate contain some common y-coordinate, if axis = CGAL_X.
     edge_coordinate is the original coordinate in the grid ogon. */
  bool Ogons_generator_2::could_edges_intersect(int fst_edge_coordinate, int snd_edge_coordinate, int axis) {

    int fst_edge_fst_vertex = line_info[axis][grid[axis][fst_edge_coordinate]].vertex_id;
    int fst_edge_snd_vertex = get_adjacent(line_info[axis][grid[axis][fst_edge_coordinate]].vertex_id,axis);

    int snd_edge_fst_vertex = line_info[axis][grid[axis][snd_edge_coordinate]].vertex_id;
    int snd_edge_snd_vertex = get_adjacent(line_info[axis][grid[axis][snd_edge_coordinate]].vertex_id,axis);

    return could_vertex_be_in_edge(fst_edge_fst_vertex,snd_edge_coordinate,axis) || could_vertex_be_in_edge(fst_edge_snd_vertex,snd_edge_coordinate,axis) ||
            could_vertex_be_in_edge(snd_edge_fst_vertex,fst_edge_coordinate,axis) || could_vertex_be_in_edge(snd_edge_snd_vertex,fst_edge_coordinate,axis);
  }

  /* Verify if, after a translation of rise units, the horizontal edge with y-coordinate edge_coordinate does not intersect any higher horizontal edge, if axis = CGAL_Y.
     Verify if, after a translation of rise units, the vertical edge with x-coordinate edge_coordinate does not intersect any higher vertical edge, if axis = CGAL_X.
     edge_coordinate is the original coordinate in the grid ogon. */
  bool Ogons_generator_2::is_colinearity_possible(int edge_coordinate, int rise, int axis) {

    for(int i=edge_coordinate+1; i<=current_dim; i++) {
      if(stretched[axis][edge_coordinate] + rise < stretched[axis][i])
        return true;
      if(could_edges_intersect(edge_coordinate,i,axis))
        return false;
    }
    return true;
  }

  /* Translate randomly the horizontal edges with y-coordinate edge_coordinate or higher (stretch randomly vertical edges), if axis = CGAL_Y.
     Translate randomly the vertical edges with x-coordinate edge_coordinate or higher (stretch randomly horizontal edges), if axis = CGAL_X.
     edge_coordinate is the original coordinate in the grid ogon. */
  void Ogons_generator_2::stretch_edges(int edge_coordinate, int* max_coordinate, int n_stretchings, int axis) {

    int difference = max_coordinate[axis]-stretched[axis][current_dim];
    
    int rise = rand()%(difference/n_stretchings+1);

    for(int i=edge_coordinate; !is_coordinate_in_grid_boundary(i); i++)
      stretched[axis][i] += rise; 
  }

  /* Let, if possible, the horizontal edge with y-coordinate edge_coordinate be collinear with higher horizontal edges
     with a probability of colinearity_probability, if axis = CGAL_Y.
     Let, if possible, the vertical edge with x-coordinate edge_coordinate be collinear with higher vertical edges
     with probability colinearity_probability, if axis = CGAL_X.
     edge_coordinate is the original coordinate in the grid ogon. */
  void Ogons_generator_2::make_colinear_edges(int edge_coordinate, float colinearity_probability, int axis) {

    int rise = stretched[axis][edge_coordinate+1] - stretched[axis][edge_coordinate];

    if(!is_colinearity_possible(edge_coordinate, rise, axis)) //no colinearity
      return;
      
    if((float)rand()/RAND_MAX<=colinearity_probability) //colinearity
      stretched[axis][edge_coordinate] += rise;
  }

  /* Stretch randomly the latest grid ogon to obtain an ogon. */
  void Ogons_generator_2::stretch_grid_ogon(int* max_coordinate, float colinearity_probability, int n_stretchings) {

    init_default_stretcher();

    int edge, axis;
    while(n_stretchings > 0) {
      edge = rand()%(current_dim-1) + 2;
      stretch_edges(edge, max_coordinate, n_stretchings, CGAL_X);
      edge = rand()%(current_dim-1) + 2;
      stretch_edges(edge, max_coordinate, n_stretchings, CGAL_Y);

      n_stretchings--;
    }

    for(int i=current_dim-1; i>0; i--) {

      axis = rand()%2; //CGAL_X or CGAL_Y        
      make_colinear_edges(i, colinearity_probability, axis);

      axis = (axis+1)%2; //the other axis
      make_colinear_edges(i, colinearity_probability, axis);
    }
  }

  /* Return the latest ogon generated. */
  std::vector< std::pair<int,int> > Ogons_generator_2::get_ogon() {

    std::vector< std::pair<int,int> > ogon;

    int first_vertex = line_info[CGAL_H][1].vertex_id;
    int i = first_vertex;
    do {
      ogon.push_back(std::pair<int,int>(stretched[CGAL_X][get_coordinate(i,CGAL_X)]-stretched[CGAL_X][1],
                                        stretched[CGAL_Y][get_coordinate(i,CGAL_Y)]-stretched[CGAL_Y][1]));
      i = vertex_info[i].prev;
    } while(i!=first_vertex);

    return ogon;
  }
}

#endif // CGAL_OGONS_GENERATOR_2_H //