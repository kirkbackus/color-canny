#include "quaternion.h"

bool operator== (const Quaternion &q1, const Quaternion &q2) {
    return(q1.a==q2.a && q1.b==q2.b && q1.c==q2.c && q1.d==q2.d);
}

bool operator!= (const Quaternion &q1, const Quaternion &q2) {
    return(q1.a!=q2.a || q1.b!=q2.b || q1.c!=q2.c || q1.d!=q2.d);
}

Quaternion operator*(const Quaternion &q2, const Quaternion &q1) {
    return(
        Quaternion(q1.a*q2.a - q1.b*q2.b - q1.c*q2.c - q1.d*q2.d,
                   q1.a*q2.b + q1.b*q2.a - q1.c*q2.d + q1.d*q2.c,
                   q1.a*q2.c + q1.b*q2.d + q1.c*q2.a - q1.d*q2.b,
                   q1.a*q2.d - q1.b*q2.c + q1.c*q2.b + q1.d*q2.a)
    );
}


Quaternion operator/(const Quaternion &q2, const Quaternion &q1) {
    float div = q1.a*q1.a+q1.b*q1.b+q1.c*q1.c+q1.d*q1.d;
    return(
        Quaternion((q1.a*q2.a - q1.b*q2.b - q1.c*q2.c - q1.d*q2.d) / div,
                   (q1.a*q2.b + q1.b*q2.a - q1.c*q2.d + q1.d*q2.c) / div,
                   (q1.a*q2.c + q1.b*q2.d + q1.c*q2.a - q1.d*q2.b) / div,
                   (q1.a*q2.d - q1.b*q2.c + q1.c*q2.b + q1.d*q2.a) / div)
    );
}
