#include <kdl/chain.hpp>
#include <kdl_format_io/urdf_import.hpp>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <kdl/frames_io.hpp>

#include <kdl/treefksolverpos_recursive.hpp>

#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_format_io/urdf_import.hpp>

#include <Eigen/Core>
#include <Eigen/LU>


#include <yarp/os/Property.h>

#include <fstream>

Eigen::Matrix<double, 9, 6> getShoulderMeasurementMatrix(KDL::TreeFkSolverPos_recursive & pos_slv, const KDL::JntArray & q)
{
    assert(q.rows() == 3);
 
    //Local relation between force/torque and measure of the 3 axis 
    Eigen::Matrix<double, 3, 6> three_axis_measure;
    three_axis_measure.setZero();
    three_axis_measure(0,2) = 1.0;
    three_axis_measure(1,3) = 1.0;
    three_axis_measure(2,4) = 1.0;

    
    //KDL::TreeFkSolverPos_recursive pos_slv(tree);
    KDL::Frame link0_frame;
    KDL::Frame link1_frame;
    KDL::Frame link2_frame;

    std::string link0_name = "link0";
    std::string link1_name = "link1";
    std::string link2_name = "link2";

    pos_slv.JntToCart(q,link0_frame,link0_name);
    pos_slv.JntToCart(q,link1_frame,link1_name);
    pos_slv.JntToCart(q,link2_frame,link2_name);

    Eigen::Matrix<double, 9, 6> return_matrix;
    
    return_matrix.block<3, 6>(0,0) = three_axis_measure*KDL::CoDyCo::WrenchTransformationMatrix(link0_frame);
    return_matrix.block<3, 6>(3,0) = three_axis_measure*KDL::CoDyCo::WrenchTransformationMatrix(link1_frame);
    return_matrix.block<3, 6>(6,0) = three_axis_measure*KDL::CoDyCo::WrenchTransformationMatrix(link2_frame);
    
    return return_matrix;
}

double getDeterminantOfShoulderMatrix(KDL::TreeFkSolverPos_recursive & pos_slv, const KDL::JntArray & q)
{
    Eigen::Matrix<double, 9, 6> shoulder_matrix;
    shoulder_matrix = getShoulderMeasurementMatrix(pos_slv,q);
    Eigen::Matrix<double, 6, 6> prod =  (shoulder_matrix.transpose()*shoulder_matrix);
    return prod.determinant();
}

int main(int argc, char ** argv)
{    
    yarp::os::Property opt;
    
    opt.fromCommand(argc,argv);
    
    if( !opt.check("urdf") ) {
        std::cout << "Usage: 3axis_kdl_analysis --urdf shoulder.urdf --step 0.01 --output output.csv" << std::endl;
    }
    
    KDL::Tree icub3shoulder;
    
    kdl_format_io::treeFromUrdfFile(opt.find("urdf").asString().c_str(),icub3shoulder);
    
    double step;
    if( opt.check("step") ) {
        step = opt.find("step").asDouble();
    } else {
        step = 0.1;
    }
    
    std::string output_filename;
    if( opt.check("output") ) {
        output_filename = opt.find("output").asString();
    } else {
        output_filename = "output.csv";
    }
    
    if( icub3shoulder.getNrOfJoints() != 3 ) { 
        std::cout << "Analysis hardcoded for 3 joints" << std::endl;
        return -1;
    }
    
    //Calculate the determinant of M^T M for all the possibol q0,q1,q2
    KDL::JntArray q(icub3shoulder.getNrOfJoints());
    
    KDL::TreeFkSolverPos_recursive pos_slv(icub3shoulder);
    
    std::ofstream output_file;
    
    output_file.open(output_filename.c_str());
    
    for(double q0=-M_PI; q0 <= M_PI; q0 += step) {
        //if( fabs(q0-ceil(q0)) < step ) {
            std::cout << "q0 " << q0 << std::endl;
        //}
        for(double q1=-M_PI; q1 <= M_PI; q1 += step) {
            for(double q2=-M_PI; q2 <= M_PI; q2 += step) {
                q(0) = q0;
                q(1) = q1;
                q(2) = q2;
                double det = getDeterminantOfShoulderMatrix(pos_slv,q);
                output_file << q0 << "," << q1 << "," << q2 << "," << det << std::endl;
            }
        }
    }
    
    output_file.close();
    
    
}
