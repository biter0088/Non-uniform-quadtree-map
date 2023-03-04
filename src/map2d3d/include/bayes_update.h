#include <math.h> //hxz
float bayes_update(float value){
  // std::cout<<"执行bayes_update函数"<<std::endl;
  // std::cout<<"value: "<<value<<std::endl;
  // std::cout<<"log(value / (1 - value))"<<log10(value / (1 - value))<<std::endl;

  // return log10(value / (1 - value));
  return log(value / (1 - value));
}