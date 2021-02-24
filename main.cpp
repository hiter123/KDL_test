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
    kdl_parser::treeFromFile("/home/kevin/CLionProjects/KDL_test/ur3_robot.urdf",my_tree);
    bool exit_value;
    Chain chain;
    exit_value = my_tree.getChain("base","tool0",chain);
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    unsigned int nj = chain.getNrOfJoints();
    JntArray jointpositions = JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf("Enter the position of joint %i: ",i);
        scanf("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }

    Frame cartpos;

    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos << std::endl;
        printf("%s \n","Success, thanks KDL!");
    }
    else{
        printf("%s \n","Error:could not calculate forward kinematics : ");
    }
}