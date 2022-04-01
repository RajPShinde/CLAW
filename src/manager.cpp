#include <Manager.hpp>

Manager::Manager(){
    begin();
}

Manager::~Manager(){
}

void Manager::begin(){
    while(true){
        state_ = getState();
        if(state_ == "WALK")
            walk();
    }
}

void Manager::walk(){
    while(state_ == "walk"){
    }
    reset();
}

void Manager::reset(){
    state_ "IDLE";
}
