#include "pidRegulater.h"

pidRegulater::pidRegulater(){
    m_pre_error = 0;
    m_cum_error = 0;
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

float pidRegulater::getResult(float currErr){
    float result;;
    m_cum_error += currErr;
    result = m_Kp*currErr + m_Ki*m_cum_error + m_Kd*m_pre_error;
    m_pre_error = currErr;
}

float pidRegulater::getResult(float currVal, float goalVal){
    float currErr;
    currErr = goalVal - currVal;
    getResult(currErr);
}