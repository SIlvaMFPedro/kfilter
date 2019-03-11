//
// Created by pedro on 03-03-2019.
//

// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           �cole Polytechnique de Montr�al
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

#ifndef PROJECT_KMATRIX_HPP
#define PROJECT_KMATRIX_HPP

//! \file
//! \brief Contains the interface of the \c KMatrix template class.

// System Includes
#include <vector>
#include <string>
#include <iostream>
#include <kfilter/ktypes.hpp>

namespace Kalman {
    template<typename T, K_UINT_32 BEG, bool DBG>
    class KMatrix {
        public:
            typedef T type;
            enum{
                beg = BEG;
            };
            inline KMatrix();
            inline KMatrix(K_UINT_32 m, K_UINT_32 n);
            inline KMatrix(K_UINT_32 m, K_UINT_32 n, const T& a);
            inline KMatrix(K_UINT_32 m, K_UINT_32 n, const T* v);
            inline KMatrix(const KMatrix& M);
            inline ~KMatrix();
            inline T& operator()(K_UINT_32 i, K_UINT_32 j);
            inline const T& operator()(K_UINT_32 i, K_UINT_32 j) const;
            inline K_UINT_32 nrow() const;
            inline K_UINT_32 ncol() const;
            inline void resize(K_UINT_32 m, K_UINT_32 n);
            inline KMatrix& operator=(const T& a);
            inline KMatrix& operator=(const KMatrix& M);
            inline void assign(K_UINT_32 m, K_UINT_32 n, const T* v);
            inline void swap(KMatrix& M);
            inline void get(std::istream& is);
            inline void put(std::ostream& os) const;

        private:
            std::vector<T*> vimpl_;
            std::vector<T> Mimpl_;
            T** M;
            K_UINT_32 m_;
            K_UINT_32 n_;
            inline void init(K_UINT_32 m, K_UINT_32 n);
    };

    template<typename T, K_UINT_32 BEG, bool DBG>
    inline std::istream& operator>>(std::istream& is, KMatrix<T, BEG, DBG>& M);

    template<typename T, K_UINT_32 BEG, bool DBG>
    inline std::istream& operator>>(std::ostream& os, const KMatrix<T, BEG, DBG>& M);

    typedef unsigned short KMatrixContext;

    extern KMatrixContext  DEFAULT_MATRIX_CONTEXT;

    KMatrixContext createKMatrixContext(std::string elemDelim = " ",
                                        std::string rowDelim = "\n",
                                        std::string startDelim = "",
                                        std::string endDelim = "",
                                        unsigned prec = 4);

    KMatrixContext selectKMatrixContext(KMatrixContext c);
}

#include <kfilter/kmatrix_impl.hpp>

#endif //PROJECT_KMATRIX_HPP
