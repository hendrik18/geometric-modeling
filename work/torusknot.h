#ifndef TORUS_KNOT_H
#define TORUS_KNOT_H

#include <parametrics/gmpcurve.h>
#include <cmath>

class TorusKnot : public GMlib::PCurve<float,3> {
    GM_SCENEOBJECT(TorusKnot)

public:
    TorusKnot() {}

protected:
    /**
     *  eval(t, d, left):
     *  - t:   parameter
     *  - d:   number of derivatives (0 => just position, 1 => +1st derivative, etc.)
     *  - left is typically not used (for one-sided derivatives).
     */
    void eval(float t, int d, bool /*left*/ = true) const override {

      // Ensure _p has room for up to d derivatives (0 => just position)
      this->_p.setDim(d + 1);

      // (p,q) for the Torus Knot
      // p = 2 twists around one axis
      // q = 3 loops through the torus hole
      float R = 2.0f;    // Major radius offset
      int   p = 2;
      int   q = 3;

      //
      // 1) Position
      //
      float x = ( R + std::cos(q * t) ) * std::cos(p * t);
      float y = ( R + std::cos(q * t) ) * std::sin(p * t);
      float z = std::sin(q * t);

      this->_p[0] = { x, y, z };


      //
      // 2) First derivative, if requested
      //
      if(d > 0) {
        // dx/dt
        //   = d/dt [ (R + cos(qt)) cos(pt ) ]
        //   = (R + cos(qt)) * -p sin(pt)  +  cos(pt) * -q sin(qt)
        float dx = -p * (R + std::cos(q * t)) * std::sin(p * t)
                   - q * std::sin(q * t)      * std::cos(p * t);

        // dy/dt
        //   = d/dt [ (R + cos(qt)) sin(pt ) ]
        //   = (R + cos(qt)) *  p cos(pt)    +  sin(pt) * -q sin(qt)
        float dy =  p * (R + std::cos(q * t)) * std::cos(p * t)
                   - q * std::sin(q * t)      * std::sin(p * t);

        // dz/dt
        //   = d/dt [ sin(qt) ] = q cos(qt)
        float dz = q * std::cos(q * t);

        this->_p[1] = { dx, dy, dz };
      }


      //
      // 3) Second derivative, if requested
      //
      if(d > 1) {
        //
        // x'(t) = -p (R + cos(qt)) sin(pt) - q sin(qt) cos(pt)
        // x''(t) = derivative of x'(t) wrt t
        //
        // Let’s do it carefully in parts:
        //   x'(t) = A(t) + B(t)
        // where
        //   A(t) = -p (R + cos(qt)) sin(pt)
        //   B(t) = -q sin(qt) cos(pt)
        //
        // A'(t) = -p [ derivative of (R+cos(qt)) sin(pt) ]
        //       = -p [ (R + cos(qt)) * p cos(pt) + sin(pt) * (-q sin(qt)) ]
        //       = -p [ p(R+cos(qt)) cos(pt) - q sin(qt) sin(pt) ]
        //
        // B'(t) = -q [ derivative of sin(qt) cos(pt) ]
        //       = -q [ q cos(qt) cos(pt) + sin(qt)*(-p sin(pt)) ]
        //       = -q [ q cos(qt) cos(pt) - p sin(qt) sin(pt) ]
        //
        // So x''(t) = A'(t) + B'(t).
        //

        float xpp = 0.0f;
        { // A'(t)
          float partA = -p * (
              p * (R + std::cos(q * t)) * std::cos(p * t)
              - q * std::sin(q * t)     * std::sin(p * t)
            );
          float partB = -q * (
              q * std::cos(q * t) * std::cos(p * t)
              - p * std::sin(q * t)* std::sin(p * t)
            );
          xpp = partA + partB;
        }

        //
        // y'(t) = p (R + cos(qt)) cos(pt) - q sin(qt) sin(pt)
        // y''(t) => similarly careful
        //
        float ypp = 0.0f;
        { // direct approach:
          // Let C(t) = p (R + cos(qt)) cos(pt)
          //     D(t) = - q sin(qt) sin(pt)
          //
          // C'(t) = p [ derivative of (R + cos(qt)) cos(pt) ]
          //       = p [ (R + cos(qt))(-p sin(pt)) + cos(pt)(-q sin(qt)) ] => factor out minus
          // D'(t) = -q [ derivative of sin(qt) sin(pt) ]
          //       = -q [ q cos(qt) sin(pt) + sin(qt)* p cos(pt) ]
          //
          float partC = p * (
              -p * (R + std::cos(q * t)) * std::sin(p * t)
              - q * std::sin(q * t)      * std::cos(p * t)
            );
          float partD = -q * (
              q * std::cos(q * t)  * std::sin(p * t)
              + p * std::sin(q * t)* std::cos(p * t)
            );
          ypp = partC + partD;
        }

        //
        // z'(t) = q cos(qt)
        // z''(t) = - q^2 sin(qt)
        //
        float zpp = - q * q * std::sin(q * t);

        this->_p[2] = { xpp, ypp, zpp };
      }
    }


    float getStartP() const override {
      // 0 to 6π so that the entire (2,3) torus knot is traced out exactly once
      return 0.0f;
    }

    float getEndP() const override {
      // For (p=2, q=3), we need 6π to get the full shape closed
      return 6.0f * float(M_PI);
    }

    bool isClosed() const override {
      return true;
    }
};

#endif // TORUS_KNOT_H
