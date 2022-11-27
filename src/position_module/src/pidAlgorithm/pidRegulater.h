#include <cmath>
#pragma once

class pidRegulater{
private:
    float m_Kp;
    float m_Ki;
    float m_Kd;
    float m_pre_error; // Previous Error
    float m_cum_error; // Accumulation Error

public:
    pidRegulater();
    void setKp(float val);
    void setKi(float val);
    void setKd(float val);
    float getKp();
    float getKi();
    float getKd();
    float getcerr();
    float getperr();
    float getResult(float currVal, float goalVal);
    float getResult(float currErr);
};