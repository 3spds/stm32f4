#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main()
{
    MatrixXd m(2,5);
    m.resize(4,3);
    std::cout << "the matrix m is of size " << m.rows() << "x" << m.cols() << std::endl;
    std::cout << "it has " << m.size() << "coefficients" << std::endl;
    VectorXd v(2);
    v.resize(5);
    std::cout << "the vector v is of size " << v.size() << std::endl;
    std::cout<< "as a matrix, v is of size " << v.rows() << "x" << v.cols() << std::endl;
}
