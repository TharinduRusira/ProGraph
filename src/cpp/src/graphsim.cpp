//
// Laplacian graph similarity implementation
// using Eigen library
//
#include "graphsim.hpp"

Eigen::MatrixXd getAdjacencyMatrix();

Eigen::MatrixXd getLaplacianEigenvalues(Eigen::MatrixXd A, Eigen::MatrixXd D){
    assert(A.rows() == A.cols() && "A should be square");
    assert(D.rows() == D.cols() && "D should be square");
    assert(A.rows() == D.rows() && "A and D dims should match");
    assert(D.isDiagonal() && "D should be diagonal");

    Eigen::MatrixXd laplacianA = D - A;
    Eigen::SelfAdjointEigenSolver<MatrixXd> es(laplacianA);
   
    return es.eigenvalues();

} 

Eigen::MatrixXd getTopKEigenvalues(MatrixXd eigenA, MatrixXd eigenB, float energy){
    assert(0.0 < energy && energy <= 1.0 && "energy must be in (0,1]");
    float cumulative_energyA = 0.0;
    float cumulative_energyB = 0.0;
    int kA = 0;
    int kB = 0;
    std::vector<float> lambdaA;
    std::vector<float> lambdaB;
    for(int i=0; i<eigenA.rows(); i++)
        lambdaA.push_back((float)eigenA(i));
    for(int i=0; i<eigenB.rows(); i++)
        lambdaB.push_back((float)eigenB(i));
    
    sort(lambdaA.begin(), lambdaA.end());
    sort(lambdaB.begin(), lambdaB.end());

    float sumA = 0;
    float sumB = 0;
    for(int i=lambdaA.size()-1; i>=0; i--)
        sumA += lambdaA[i];
    float meanA = sumA/lambdaA.size();

    for(int i=lambdaB.size()-1; i>=0; i--)
        sumB += lambdaB[i];
    float meanB = sumB/lambdaB.size();
    
    sumA = 0;
    sumB = 0;
    for(int i=lambdaA.size()-1; i>=0; i--){
        sumA += lambdaA[i];
        if(sumA > meanA*energy){
            kA = i;
            break;
        }
    }
    for(int i=lambdaB.size()-1; i>=0; i--){
        sumB += lambdaB[i];
        if(sumB > meanB*energy){
            kB = i;
            break;
        }
    }
    int mink = std::min(kA, kB);
    Eigen::MatrixXd res(mink, 2); // top-mink eigenvalues for A and B

    for(int i=lambdaA.size()-1; i>=lambdaA.size()-mink; i--)
        res(lambdaA.size() - 1 - i, 0) = lambdaA[i];

    for(int i=lambdaB.size()-1; i>=lambdaB.size()-mink; i--)
        res(lambdaB.size() - 1 - i, 1) = lambdaB[i];

    return res;
}

float similarity(Eigen::MatrixXd topkmin){
    float sim = 0;
    for(int i=0; i<topkmin.rows();i++)
        sim += (topkmin(i,0) - topkmin(i, 1))*(topkmin(i,0) - topkmin(i, 1));
    return sim;
}

