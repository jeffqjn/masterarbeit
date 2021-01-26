#include "angular_v.h"

void Parameters::set_GYRO_SCALE_FACTOR(double value)
{
    this->GYRO_SCALE_FACTOR=value;
}
void Parameters::set_GYRO_BIAS(double value)
{
    this->GYRO_BIAS=value;
}
void Parameters::set_Gyro_WALKING_BIAS(double value)
{
    this->Gyro_WALKING_BIAS=value;
}
void Parameters::set_START_COMPUTE_TIME(double value)
{
    this->START_COMPUTE_TIME=value;
}
void Parameters::set_END_COMPUTE_TIME(double value)
{
    this->END_COMPUTE_TIME=value;
}
void Parameters::set_Bag_Path(string path) {
    this->BAG_PATH=path;
}
double Parameters::get_GYRO_SCALE_FACTOR()
{
    return Parameters::GYRO_SCALE_FACTOR;
}
double Parameters::get_GYRO_BIAS()
{
    return Parameters::GYRO_BIAS;
}
double Parameters::get_Gyro_WALKING_BIAS()
{
    return Parameters::Gyro_WALKING_BIAS;
}
double Parameters::get_START_COMPUTE_TIME()
{
    return Parameters::START_COMPUTE_TIME;
}
double Parameters::get_END_COMPUTE_TIME()
{
    return Parameters::END_COMPUTE_TIME;
}
string Parameters::get_Bag_Path() {
    return Parameters::BAG_PATH;
}


//
// Created by noah on 23.01.21.
//

