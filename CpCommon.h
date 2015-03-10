#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <fstream>
#include <vector>
#include <sys/time.h>
// Aux Time Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double Tic() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double RealTime() {
    return Tic();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double Toc(double dTic) {
    return Tic() - dTic;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double TocMS(double dTic) {
    return ( Tic() - dTic)*1000.;
}

////////////////////////////////////////////////////////////////////////////

template <class Derived> std::fstream& operator>>(std::fstream& fs, Eigen::MatrixBase<Derived>& mM) {
    typedef typename Eigen::internal::traits<Derived>::Scalar LScalar;
    std::string sLine;
    int nHeight = 0;
    int nWidth;
    std::vector<LScalar> vVals;
    LScalar dVal;
    while (getline(fs, sLine)) {
        std::stringstream ss(std::stringstream::in |
                std::stringstream::out);
        ss << sLine;
        while (ss >> dVal) {
            vVals.push_back(dVal);
        }
        if (nHeight == 0) nWidth = vVals.size();
        nHeight++;
    }
    if (int(vVals.size()) != nWidth * nHeight) {
        printf("ERROR: while reading matrix from file, missing data.\n");
        return fs;
    }
    mM = Derived(nHeight, nWidth);
    for (int nRow = 0; nRow < nHeight; nRow++) {
        for (int nCol = 0; nCol < nWidth; nCol++) {
            mM(nRow, nCol) = vVals[ nRow * nWidth + nCol ];
        }
    }
    return fs;
}

#endif // COMMON_H
