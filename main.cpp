/*#include <iostream>
#include <kdl/frames.hpp>

//using namespace KDL;
using namespace std;
int main(int argc, char** argv)
{
//    Vector v = Vector(1,2,3);
    KDL::Vector v1(1,2,3); //隐式创建对象
    for(int i=0; i<3; ++i){
        cout<<v1[i]<<";";
    }

    cout<<"\n";
    KDL::Vector v0 = KDL::Vector::Zero(); //显式创建对象
    KDL::Vector vec; //隐式创建对象，调用默认构造函数，成员数据被初始化为0
    cout<<vec.x()<<";"<<vec.y()<<";"<<vec.z()<<"\n";
//    std::cout<<v.data[0]<<std::endl; //获取数组的第一个元素
    for(int i=0; i<3; ++i){
        cout<<v0(i)<<" "; //循环输出数组的元素
    }
    cout<<"\n";
    cout<<v0.x()<<" "; //获取数组的第一个元素
    cout<<v0[0]; //获取数组的第一个元素
    return 0;
}*/
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

using namespace KDL;
using namespace std;
int main(int argc,char** argv){
    Tree my_tree;
    kdl_parser::treeFromFile("/home/kevin/CLionProjects/KDL_test/nimbro_adult.urdf",my_tree);
    bool exit_value;
    Chain chain_left,chain_right ;
    exit_value = my_tree.getChain("trunk_link","left_foot_plane_link",chain_left);
    exit_value = my_tree.getChain("trunk_link","right_foot_plane_link",chain_right);
    ChainFkSolverPos_recursive fksolver_left = ChainFkSolverPos_recursive(chain_left);
    ChainFkSolverPos_recursive fksolver_right = ChainFkSolverPos_recursive(chain_right);
    std::vector<Segment> my_segments_left=chain_left.segments;
    std::vector<Segment> my_segments_right=chain_right.segments;
    for (auto j = my_segments_left.begin(); j < my_segments_left.end(); ++j) {
        cout<<j->getName()<<endl;
    }
    for (auto j = my_segments_right.begin(); j < my_segments_right.end(); ++j) {
        cout<<j->getName()<<endl;
    }
    unsigned int nj_left = chain_left.getNrOfJoints();
    unsigned int nj_right = chain_right.getNrOfJoints();
    JntArray jointpositions_left = JntArray(nj_left);
    JntArray jointpositions_right = JntArray(nj_right);

    for(unsigned int i=0;i<nj_left;i++){
        float myinput_left;
        printf("Enter the position of joint %i: ",i);
        scanf("%e",&myinput_left);
        jointpositions_left(i)=(double)myinput_left;
    }
    for(unsigned int i=0;i<nj_right;i++){
        float myinput_right;
        printf("Enter the position of joint %i: ",i);
        scanf("%e",&myinput_right);
        jointpositions_right(i)=(double)myinput_right;
    }
    Frame cartpos_left,cartpos_right;

    bool kinematics_status_left,kinematics_status_right;
    kinematics_status_left = fksolver_left.JntToCart(jointpositions_left,cartpos_left);
    kinematics_status_right= fksolver_right.JntToCart(jointpositions_right,cartpos_right);
    if(kinematics_status_left>=0){
        std::cout << cartpos_left << std::endl;
        printf("%s \n","Success, thanks KDL!");
    }
    else{
        printf("%s \n","Error:could not calculate forward kinematics : ");
    }
    if(kinematics_status_right>=0){
        std::cout << cartpos_right << std::endl;
        printf("%s \n","Success, thanks KDL!");
    }
    else{
        printf("%s \n","Error:could not calculate forward kinematics : ");
    }
    ///
}