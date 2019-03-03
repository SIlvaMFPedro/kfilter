//
// Created by pedro on 03-03-2019.
//

#ifndef PROJECT_SIMPLE_PLANE_H
#define PROJECT_SIMPLE_PLANE_H

#include <kfilter/ekfilter.hpp>


class cPlaneEKF_sp : public Kalman::EKFilter<double,1>
{
    public:
        cPlaneEKF_sp();

    protected:
        void makeA();
        void makeH();
        void makeV();
        void makeR();
        void makeW();
        void makeQ();
        void makeProcess();
        void makeMeasure();

        double Period, Mass, Bfriction, Portance, Gravity;
};

typedef cPlaneEKF_sp::Vector Vector;
typedef cPlaneEKF_sp::Matrix Matrix;

#endif //PROJECT_SIMPLE_PLANE_H
