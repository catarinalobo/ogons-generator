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


#ifndef CGAL_GRID_OGON_2_H
#define CGAL_GRID_OGON_2_H

#include <CGAL/Arrangement_2.h>
#include <vector>
#include <list>


namespace CGAL {
  
  class Grid_ogon_2 {

  protected:
    std::vector< std::pair<int,int> > vertices;

	public:
		Grid_ogon_2() {};
  	Grid_ogon_2(std::vector< std::pair<int,int> > vertices);
    Grid_ogon_2(const Grid_ogon_2& grid_ogon);
    
    friend void swap(Grid_ogon_2& fst, Grid_ogon_2& snd) {
      swap(fst.vertices, snd.vertices);
    }
    Grid_ogon_2& operator=(Grid_ogon_2 grid_ogon) {
      swap(*this, grid_ogon);
      return *this;
    }

		template<typename Arrangement_2_> Arrangement_2_ get_arrangement();
		template<typename Point_2_> std::list<Point_2_> get_list();
  };

  /* Constructor */
  Grid_ogon_2::Grid_ogon_2(std::vector< std::pair<int,int> > vertices) {

  	this->vertices = vertices;
  }

  /* Copy Constructor */
  Grid_ogon_2::Grid_ogon_2(const Grid_ogon_2& grid_ogon) {

    this->vertices = grid_ogon.vertices;
  }


  template<typename Arrangement_2_> Arrangement_2_ Grid_ogon_2::get_arrangement() {

  	typedef Arrangement_2_                                Arrangement_2;
    typedef typename Arrangement_2::Geometry_traits_2     Geometry_traits_2;
    typedef typename Geometry_traits_2::Point_2           Point_2;
    typedef typename Geometry_traits_2::Segment_2         Segment_2;

    Arrangement_2 ogon;

    std::vector<Segment_2> segments;
    for(int i=0; i<vertices.size()-1; i++) {
      segments.push_back(Segment_2(Point_2(vertices[i].first, vertices[i].second),
                                   Point_2(vertices[i+1].first, vertices[i+1].second)));
    }
    segments.push_back(Segment_2(Point_2(vertices[vertices.size()-1].first, vertices[vertices.size()-1].second),
                                   Point_2(vertices[0].first, vertices[0].second)));

    insert_non_intersecting_curves(ogon, segments.begin(), segments.end());

    return ogon;
  }

  template<typename Point_2_> std::list<Point_2_> Grid_ogon_2::get_list() {

  	typedef Point_2_   Point_2;

    std::list<Point_2> ogon;
    for(int i=0; i<vertices.size(); i++) {
      ogon.push_back(Point_2(vertices[i].first, vertices[i].second));
    }
    
    return ogon;
  }
}

#endif // CGAL_GRID_OGON_2_H //