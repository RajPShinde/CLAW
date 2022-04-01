#include <polynomial.hpp>

Polynomial::Polynomial(double x0, double v0, double a0, double xT, double vT, double aT, double T):
					   x0_(x0), v0_(v0), a0_(a0), xT_(xT), vT_(vT), aT_(aT), T_(T), a1_(x0), a2_(v0), a3_(a0/2) {
	Eigen::Matrix3d A;
	Eigen::Vector3d B;
	A << std::pow(T_,3), std::pow(T_,4), std::pow(T_,5),
		 3 * std::pow(T_,2), 4 * std::pow(T_,3), 5 * std::pow(T_,4),
		 6 * T_, 12 * std::pow(T_,2), 20 * std::pow(T_,3);

	B << xT - a1_ - a2_ * T_ - a3_ * std::pow(T_,2),
		 vT - a2_ - 2 * a3_ * T_,
		 aT - 2 * a3_;

	//  Solve for x in Ax=B
	Eigen::Vector3d coefficients = A.colPivHouseholderQr().solve(B);
	a4_ = coefficients(0); 
	a5_ = coefficients(1);
	a6_ = coefficients(2);
}

Polynomial::~Polynomial(){
}

Polynomial::position(){

}

Polynomial::velocity(){

}

Polynomial::acceleration(){

}

Polynomial::jerk(){
    
}