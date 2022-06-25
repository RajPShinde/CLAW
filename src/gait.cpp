#include <gait.hpp>

Gait::Gait(double s, double h, double e, double m, std::string type): stride(s), height(h), te(e), tm(m), tn(tm-2*te), name(type){
}

Gait::~Gait(){
}