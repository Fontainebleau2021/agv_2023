



#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <limits>

int main()
{

    std::pair<double, double> k1(9,4);
    std::pair<double, double> k2(3,2);
    std::pair<double, double> k3(4,1);
    std::pair<double, double> k4(3,2);
    bool ans = k2<k1;
    std::cout<<ans<<std::endl;
    ans = k2<k3;
    std::cout<<ans<<std::endl;
    ans = k2>k4;
    std::cout<<ans<<std::endl;
    // *n2 = 0;
    // //open_pq_.emplace(n4);
    // //open_pq_.emplace()
    // std::cout<<*open_pq_.top()<<std::endl;
    // //std::cout<<n1.use_count();
    return 0;
}