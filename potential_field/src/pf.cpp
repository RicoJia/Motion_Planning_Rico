//
// Created by ricojia on 5/23/20.
//

#include "../include/potential_field/pf.hpp"

using std::cout;
using std::endl;
using std::vector;

namespace potential_field{
    using rigid2d::Vector2D;
    using rigid2d::distance;
    using rigid2d::length;
    using PRM_Grid::Vertex;

    pf::pf() {}

    pf::pf(double cell_size,
           XmlRpc::XmlRpcValue obstacle_list,
           double bounding_r,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           const std::vector<double>& map_x_lims,
           const std::vector<double>& map_y_lims
           ):m_prm(),
             m_bounding_r (bounding_r),
             m_start({start.at(0), start.at(1)}),
             m_goal({goal.at(0), goal.at(1)}),
             m_map_x_lims(map_x_lims),
             m_map_y_lims(map_y_lims),
             m_current_pos({start.at(0), start.at(1)}),
             m_epsilon(0.15),
             m_ksi(0.05),
             m_eta(0.1),
             m_q_star(0.3),
             m_d_star(0.15)
    {
        m_prm.add_obstacles_and_normal_vecs(obstacle_list, cell_size);
    }

    //================================interface funcs==========================
    PRM_Grid::Vertex pf::get_next_waypoint(){
        Vector2D att_gradient = this -> get_att_gradient();
        Vector2D rep_gradient = this -> get_rep_gradient();
        auto Dn = (att_gradient + rep_gradient);
        Dn.normalize_vec();
        m_current_pos -= m_ksi * Dn;
        if(this -> is_in_obstacle()){
            ROS_FATAL("Robot is in obstacle. Potential Field Planning Node has stopped");
            throw std::runtime_error("");
        }
        if(this -> has_reached_goal()){
            ROS_INFO("Robot has reached goal. Potential Field Planning Node has stopped");
            throw std::runtime_error("");
        }
        return {m_current_pos.x, m_current_pos.y};
    }

    //================================private funcs==========================
    rigid2d::Vector2D pf::get_att_gradient() const{
        Vector2D gradient;
        double dist = distance(m_current_pos, m_goal);
        if(dist <= m_d_star){
            gradient = m_ksi * (m_current_pos - m_goal);
        }
        else{
            gradient = m_d_star * m_ksi * (m_current_pos - m_goal) *  (1.0/ dist);
        }
        return gradient;
    }

    rigid2d::Vector2D pf::get_rep_gradient() const{
        Vector2D gradient;
        vector<Vertex> closest_pts = m_prm.get_closest_pts_from_obstacles({m_current_pos.x, m_current_pos.y});

        // calculate rep gradient from each obstacle point
        for(Vertex& obstacle_pt: closest_pts){
            Vector2D dist { m_current_pos.x - obstacle_pt.coord.x, m_current_pos.y - obstacle_pt.coord.y};
            double dq = length(dist);
            if(dq <= m_q_star){
                dist.normalize_vec();
                gradient += (m_eta * (1.0/m_q_star - 1.0/dq) * (1.0/(dq*dq)) * dist);
            }
        }
        return gradient;
    }

    //test code for get_rep_gradient()
//    Vertex pos_vertex[] {{1.0 * 0.2, 4.0 * 0.2}, {2.0 * 0.2, 4.0 * 0.2}, {4.0 * 0.2,4.0 * 0.2},
//                         {5.0 * 0.2, 10.0 * 0.2}, {5.0 * 0.2, 20.0 * 0.2}, {5.0 * 0.2, 25.0 * 0.2},
//                         {1.0 * 0.2, 27.0 * 0.2}, {2.0 * 0.2, 27.0 * 0.2}, {4.0 * 0.2, 27.0 * 0.2}};
//    for (Vertex& pos_v: pos_vertex){
//    vector<Vertex> closest_pts = m_prm.get_closest_pts_from_obstacles(pos_v);
//    for (Vertex& closest_pt : closest_pts){
//    cout<<"closest pt: x,y: "<< closest_pt.coord.x * 5 <<", "<<closest_pt.coord.y * 5<<endl;
//}
//cout<<"=================="<<endl;
//}

    bool pf::has_reached_goal() const{
        return distance(m_current_pos, m_goal) < m_epsilon;
    }

    bool pf::is_in_obstacle() const{
        return m_prm.if_in_obstacle({m_current_pos.x, m_current_pos.y});
    }
}