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
 /* Last update: 20/02/2017                                                            */
/**************************************************************************************/


#ifndef CGAL_COLUMN_CONVEX_GRID_OGONS_2_H
#define CGAL_COLUMN_CONVEX_GRID_OGONS_2_H

#include "row_convex_grid_ogons_2.h"


namespace CGAL {

  /* Generate a row-convex grid ogon by Inflate-Paste Algorithm. */
  Grid_ogon_2 Grid_ogons_generator_2::generate_column_convex_grid_ogon() {
    
    generate_row_convex_grid_ogon();

    //Swap lines
    int swap;
    for(int i=1; i<=current_dim; i++) {

      swap = line_info[CGAL_H][i].coordinate;
      line_info[CGAL_H][i].coordinate = line_info[CGAL_V][i].coordinate;
      line_info[CGAL_V][i].coordinate = swap;

      swap = line_info[CGAL_H][i].vertex_id;
      line_info[CGAL_H][i].vertex_id = line_info[CGAL_V][i].vertex_id;
      line_info[CGAL_V][i].vertex_id = swap;

      swap = grid[CGAL_H][i];
      grid[CGAL_H][i] = grid[CGAL_V][i];
      grid[CGAL_V][i] = swap;
    }
    for(int i=0; i<n_vertices; i++) {

      swap = vertex_info[i].line_id[CGAL_H];
      vertex_info[i].line_id[CGAL_H] = vertex_info[i].line_id[CGAL_V];
      vertex_info[i].line_id[CGAL_V] = swap;
    }

    return get_grid_ogon();
  }
}

#endif // CGAL_COLUMN_CONVEX_GRID_OGONS_2_H //