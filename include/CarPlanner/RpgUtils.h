#ifndef _MISC_UTILS_H_
#define _MISC_UTILS_H_
#include <Eigen/Eigen>

namespace rpg
{
    inline float AngleWrap( float d )
    {
        while( d > M_PI ) {
            d -= 2*M_PI;
        }
        while( d < -M_PI ) {
            d += 2*M_PI;
        }
        return d;
    }

    ///////////////////////////////////////////////////////////////////////
    inline Eigen::Matrix3f	TInv( const Eigen::Matrix3f& T )
    {
        Eigen::Matrix3f Tinv;
        Tinv(0, 0) = T(0, 0);
        Tinv(0, 1) = T(1, 0);
        Tinv(1, 1) = T(1, 1);
        Tinv(1, 0) = T(0, 1);
        Tinv(0, 2) = -( T(0, 0) * T(0, 2) + T(1, 0) * T(1, 2) );
        Tinv(1, 2) = -( T(0, 1) * T(0, 2) + T(1, 1) * T(1, 2) );
        Tinv(2, 0) = 0;
        Tinv(2, 1) = 0;
        Tinv(2, 2) = 1;

        return Tinv;
    }

    ///////////////////////////////////////////////////////////////////////
    inline Eigen::Matrix3f Cart2T( float x, float y, float theta )
    {
        Eigen::Matrix3f T;

        T(0, 0) =  cos(theta);
        T(0, 1) = -sin(theta);
        T(1, 0) =  sin(theta);
        T(1, 1) =  cos(theta);
        T(0, 2) =  x;
        T(1, 2) =  y;
        T(2, 0) =  0;
        T(2, 1) =  0;
        T(2, 2) =  1;
        return T;
    }

    ///////////////////////////////////////////////////////////////////////
    inline Eigen::Vector3f R2Cart( Eigen::Matrix3f R )
    {
        Eigen::Vector3f Cart_angles;

        Cart_angles(0) = atan2(R(2,1),R(2,2));
        Cart_angles(1) = atan2(-R(2,0),sqrt(pow(R(2,1),2)+pow(R(2,2),2)));
        Cart_angles(2) = atan2(R(1,0),R(0,0));
        return Cart_angles;
    }
}

#endif

