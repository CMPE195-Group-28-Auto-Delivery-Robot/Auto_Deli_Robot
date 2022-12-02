#include "pidRegulater.h"

pidRegulater::pidRegulater(){
    Clear();
}

void pidRegulater::setKp(float val){
    m_Kp=val;
}
void pidRegulater::setKi(float val){
    m_Ki=val;
}
void pidRegulater::setKd(float val){
    m_Kd=val;
}

float pidRegulater::getKp(){
    return m_Kp;
}
float pidRegulater::getKi(){
    return m_Ki;
}
float pidRegulater::getKd(){
    return m_Kd;
}
float pidRegulater::getcerr(){
    return m_cum_error;
}
float pidRegulater::getperr(){
    return m_pre_error;
}

void pidRegulater::Clear(){
    m_pre_error = 0;
    m_cum_error = 0;
}

float pidRegulater::getResult(float currErr){
    float result;
    result = m_Kp*currErr + m_Ki*m_cum_error + m_Kd*m_pre_error;
    m_cum_error += currErr;
    m_pre_error = currErr;
    return result;
}

float pidRegulater::getResult(float currVal, float goalVal){
    float currErr;
    currErr = goalVal - currVal;
    return getResult(currErr);
}