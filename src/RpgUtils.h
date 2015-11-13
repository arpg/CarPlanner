#ifndef _MISC_UTILS_H_
#define _MISC_UTILS_H_

namespace rpg
{
    inline double AngleWrap( double d )
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
    inline Eigen::Matrix3d  TInv( const Eigen::Matrix3d& T )
    {
        Eigen::Matrix3d Tinv;
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
    inline Eigen::Matrix3d Cart2T( double x, double y, double theta )
    {
        Eigen::Matrix3d T;

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
    inline Eigen::Vector3d R2Cart( Eigen::Matrix3d R )
    {
        Eigen::Vector3d Cart_angles;

        Cart_angles(0) = atan2(R(2,1),R(2,2));
        Cart_angles(1) = atan2(-R(2,0),sqrt(pow(R(2,1),2)+pow(R(2,2),2)));
        Cart_angles(2) = atan2(R(1,0),R(0,0));
        return Cart_angles;
    }
}

#endif
