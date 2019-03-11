//
// Created by pedro on 07-03-2019.
//

// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           École Polytechnique de Montréal
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef PROJECT_EKFILTER_HPP
#define PROJECT_EKFILTER_HPP

/// \file
/// \brief Contains the interface of the \c EKFilter base template class.

// System Includes
#include <kfilter/kvector.hpp>
#include <kfilter/kmatrix.hpp>

namespace Kalman {
    template<typename T, K_UINT_32 BEG, bool OQ = false, bool OVR = false, bool DBG = true>

    class EKFilter {
        public:
            typedef T type;
            enum{
                beg = BEG;
            };
            typedef KVector<T, BEG, DBG> Vector;
            typedef KMatrix<T, BEG, DBG> Matrix;

            EKFilter();
            EKFilter(K_UINT_32 n_, K_UINT_32 nu_, K_UINT_32 nw_, K_UINT_32 m_, K_UINT_32 nv_);
            virtual ~EKFilter();

            K_UINT_32 getSizeX() const;
            K_UINT_32 getSizeU() const;
            K_UINT_32 getSizeW() const;
            K_UINT_32 getSizeZ() const;
            K_UINT_32 getSizeV() const;

            // TODO !!! watch out : i don't know which dims can be 0 ?
            void setDim(K_UINT_32 n_, K_UINT_32 nu_, K_UINT_32 nw_, K_UINT_32 m_, K_UINT_32 nv_);
            void setSizeX(K_UINT_32 n_);
            void setSizeU(K_UINT_32 nu_);
            void setSizeW(K_UINT_32 nw_);
            void setSizeZ(K_UINT_32 m_);
            void setSizeV(K_UINT_32 nv_);

            void init(Vector& x_, Matrix& P_);
            void step(Vector& u_, const Vector& z_);
            void timeUpdateStep(Vector& u_);
            void measureUpdateStep(Vector& u_);

            const Vector& predict(Vector& u_);
            const Vector& simulate();
            const Vector& getX() const;
            const Matrix& calculateP() const;

        protected:
            void NoModification();

            // TODO !!! watch out for all virtual functions : can dims be 0 ?
            virtual void makeBaseA();
            virtual void makeBaseW();
            virtual void makeBaseQ();
            virtual void makeBaseH();
            virtual void makeBaseV();
            virtual void makeBaseR();
            virtual void makeCommonProcess();
            virtual void makeA();
            virtual void makeW();
            virtual void makeQ();
            virtual void makeProcess() = 0;
            virtual void makeCommonMeasure();
            virtual void makeH();
            virtual void makeV();
            virtual void makeR();
            virtual void makeMeasure() = 0;
            virtual void makeDZ();
            virtual void sizeUpdate();

            Vector x;
            Vector u;
            Vector z;
            Vector dz;

            Matrix A;
            Matrix W;
            Matrix Q;
            Matrix H;
            Matrix V;
            Matrix R;

            K_UINT_32 n;
            K_UINT_32 nu;
            K_UINT_32 nw;
            K_UINT_32 m;
            K_UINT_32 nv;

        private:
            static void factor(Matrix& P_);
            static void upperInvert(Matrix& P_);

            void timeUpdate();
            void measureUpdate(T dz, T r);
            void makeBaseAImpl();
            void makeBaseWImpl();
            void makeBaseQImpl();
            void makeBaseHImpl();
            void makeBaseVImpl();
            void makeBaseRImpl();
            void makeAImpl();
            void makeWImpl();
            void makeQImpl();
            void makeHImpl();
            void makeVImpl();
            void makeRImpl();

            Matrix U;
            Matrix W_;
            Matrix Q_;
            Matrix H_;
            Matrix R_;

            Vector a;
            Vector d;
            Vector v;

            K_UINT_32 nn;
            mutable Matrix _P;
            mutable Vector _x;

            K_UINT_16 kflags;
            bool modified_;

    };
}

#include <kfilter/ekfilter_impl.hpp>

#endif //PROJECT_EKFILTER_HPP
