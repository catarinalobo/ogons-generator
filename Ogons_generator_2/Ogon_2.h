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


#ifndef CGAL_OGON_2_H
#define CGAL_OGON_2_H

#include "Grid_ogons_generator_2/Grid_ogon_2.h"


namespace CGAL {
  
	class Ogon_2: public Grid_ogon_2 {

  protected:
    Grid_ogon_2 grid_ogon;

  public:
  	Ogon_2() {};
  	Ogon_2(Grid_ogon_2 grid_ogon, std::vector< std::pair<int,int> > vertices);
    Ogon_2(const Ogon_2& ogon);

    friend void swap(Ogon_2& fst, Ogon_2& snd) {
      swap(fst.vertices, snd.vertices);
      swap(fst.grid_ogon, snd.grid_ogon);
    }
    Ogon_2& operator=(Ogon_2 ogon) {
      swap(*this, ogon);
      return *this;
    }

  	Grid_ogon_2 get_grid_ogon();
  };

  /* Constructor */
  Ogon_2::Ogon_2(Grid_ogon_2 grid_ogon, std::vector< std::pair<int,int> > vertices): Grid_ogon_2(vertices) {

  	this->grid_ogon = grid_ogon;
  }

  /* Copy Constructor */
  Ogon_2::Ogon_2(const Ogon_2& ogon) {

    this->vertices = ogon.vertices;
    this->grid_ogon = ogon.grid_ogon;
  }

  Grid_ogon_2 Ogon_2::get_grid_ogon() {

  	return grid_ogon;
  }    
}

#endif // CGAL_OGON_2_H //