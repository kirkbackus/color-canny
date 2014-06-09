#include <math.h>

class Quaternion
{
    private:

    union {
        struct {
            float a,b,c,d;
        };

        float v[4];
    };

    public:

    Quaternion() : a(0), b(0), c(0), d(0) {}
    Quaternion(float a, float b, float c, float d) : a(a), b(b), c(c), d(d) {}
    Quaternion(const Quaternion& q) { a = q.a; b = q.b; c = q.c; d = q.d; }

    float modulus() { return(sqrt(pow(a,2)+pow(b,2)+pow(c,2)+pow(d,2))); }
    Quaternion conjugate() { Quaternion q(a, -b, -c, -d); return(q); }
    Quaternion inverse() { float div = a*a+b*b+c*c+d*d; Quaternion q(a/div, b/div, c/div, d/div); return(q); }

    /*float[3] rotate(float[3] vec) {
        float[3] vecprime = {0,0,0};
        return(vecprime);
    }*/

    Quaternion & operator=(const Quaternion &rq) {
        if (this == &rq) return(*this);
        a = rq.a; b = rq.b; c = rq.c; d = rq.d;
        return(*this);
    }

    Quaternion & operator+=(const Quaternion &rq) {
        a += rq.a; b += rq.b; c += rq.c; d += rq.d;
        return(*this);
    }

    Quaternion & operator-=(const Quaternion &rq) {
        a -= rq.a; b -= rq.b; c -= rq.c; d -= rq.d;
        return(*this);
    }

    friend bool operator== (const Quaternion &q1, const Quaternion &q2);
    friend bool operator!= (const Quaternion &q1, const Quaternion &q2);

    friend Quaternion operator*(const Quaternion &q1, const Quaternion &q2);
    friend Quaternion operator/(const Quaternion &q1, const Quaternion &q2);

    Quaternion operator*(const float num) const { Quaternion q(a*num, b*num, c*num, d*num); return(q); }
    Quaternion operator/(const float num) const { Quaternion q(a/num, b/num, c/num, d/num); return(q); }
    Quaternion operator+(const Quaternion &q) { return(Quaternion(a+q.a, b+q.b, c+q.c, d+q.d)); }
    Quaternion operator-(const Quaternion &q) { return(Quaternion(a-q.a, b-q.b, c-q.c, d-q.d)); }
};
