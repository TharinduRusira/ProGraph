#include "graphsim.hpp"


int sameMatrixTest(){
    MatrixXd m(3,3);
    m(0,0) = 3;
    m(1,0) = 2;
    m(2,0) = 1;
    
    m(0,1) = 1;
    m(1,1) = m(1,0) + m(0,1);
    m(1,2) = 5;

    m(0,2) = 4;
    m(1,2) = 60;
    m(2,2) = 3;

    MatrixXd d(3,3);
    d(0,0) = 30;
    d(1,0) = 0;
    d(2,0) = 0;

    d(0,1) = 0;
    d(1,1) = 1;
    d(2,1) = 0;

    d(0,2) = 0;
    d(1,2) = 0;
    d(2,2) = 100;

    MatrixXd res =  getLaplacianEigenvalues(m, d);
    MatrixXd topkmin = getTopKEigenvalues(res, res, 0.9);
    assert(similarity(topkmin) == 0 && "Test failed");

    return 0;

}

int main(){
    sameMatrixTest();
}

