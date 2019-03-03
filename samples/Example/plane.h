//
// Created by pedro on 03-03-2019.
//

#ifndef PROJECT_PLANE_H
#define PROJECT_PLANE_H

#include <kfilter/ekfilter.hpp>

class cPlaneEKF : public Kalman::EKFilter<double,1,false,true,false> {
    public:
        cPlaneEKF();

    protected:
        void makeBaseA();
        void makeBaseH();
        void makeBaseV();
        void makeBaseR();
        void makeBaseW();
        void makeBaseQ();

        void makeA();
        void makeH();
        void makeProcess();
        void makeMeasure();

        double Period, Mass, Bfriction, Portance, Gravity;
};

typedef cPlaneEKF::Vector Vector;
typedef cPlaneEKF::Matrix Matrix;


#endif //PROJECT_PLANE_H
