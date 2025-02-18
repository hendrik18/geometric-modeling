#include <parametrics/gmpcurve.h>

#include <core/containers/gmdvector.h>

class MyB_spline : public GMlib::PCurve<float,3> {
    GM_SCENEOBJECT(MyB_spline)

public:
    // Constructor 1: Using given control points
    MyB_spline(const GMlib::DVector<GMlib::Vector<float,3>>& c);
    
    // Constructor 2: Using least squares to determine control points
    MyB_spline(const GMlib::DVector<GMlib::Vector<float,3>>& p, int n);

    protected:
    void eval(float t, int d, bool left = true) const override;
    float getStartP() const override;
    float getEndP() const override;
    bool isClosed() const override;

private:
    GMlib::DVector<GMlib::Vector<float,3>> _controlPoints;
    GMlib::DVector<float> _knotVector;

    void generateKnotVector();
    void leastSquaresFit(const GMlib::DVector<GMlib::Vector<float,3>>& p, int n);
        float evaluateBasis(int i, int k, float t) const;

};

// Constructor implementation
MyB_spline::MyB_spline(const GMlib::DVector<GMlib::Vector<float,3>>& c)
    : _controlPoints(c) {
    generateKnotVector();
}

// Constructor for least squares approximation
MyB_spline::MyB_spline(const GMlib::DVector<GMlib::Vector<float,3>>& p, int n) {
    leastSquaresFit(p, n);
    generateKnotVector();
}

// Generate a uniform knot vector for a 2nd-degree B-spline
void MyB_spline::generateKnotVector() {
    int n = _controlPoints.getDim(); // Number of control points
    int k = 2; // Degree
    int m = n + k + 1; // Number of knots

    _knotVector.setDim(m);
    
    // First k+1 knots are 0
    for (int i = 0; i <= k; ++i) {
        _knotVector[i] = 0.0f;
    }
    
    // Middle knots are uniformly spaced
    for (int i = k + 1; i < m - (k + 1); ++i) {
        _knotVector[i] = static_cast<float>(i - k);
    }
    
    // Last k+1 knots are max value
    float maxValue = static_cast<float>(m - 2 * (k + 1) + 1);
    for (int i = m - (k + 1); i < m; ++i) {
        _knotVector[i] = maxValue;
    }
}

// Least squares fitting to compute control points
void MyB_spline::leastSquaresFit(const GMlib::DVector<GMlib::Vector<float,3>>& p, int n) {
    int m = p.getDim(); // Number of input points
    int k = 2; // Degree of B-spline

    // Set up the control points storage
    _controlPoints.setDim(n);

    // Solve least squares system using normal equations
    GMlib::DMatrix<float> N(m, n, 0.0f);
    GMlib::DVector<float> weights(m, 1.0f);

    // Compute basis functions for each input point
    for (int i = 0; i < m; ++i) {
        float t = static_cast<float>(i) / (m - 1);
        int span = -1;
        for (int j = k; j < n; ++j) {
            if (t >= _knotVector[j] && t < _knotVector[j + 1]) {
                span = j;
                break;
            }
        }
        if (span == -1) span = n - 1;

        GMlib::DVector<float> N_i(k + 1, 0.0f);
        N_i[0] = 1.0f;
        for (int d = 1; d <= k; ++d) {
            float left = t - _knotVector[span + 1 - d];
            float right = _knotVector[span + d] - t;
            float saved = 0.0f;
            
            for (int r = 0; r < d; ++r) {
                float temp = N_i[r] / (_knotVector[span + r + 1] - _knotVector[span + 1 - d + r]);
                N_i[r] = saved + right * temp;
                saved = left * temp;
            }
            N_i[d] = saved;
        }
        
        for (int j = 0; j <= k; ++j) {
            N[i][span - k + j] = N_i[j];
        }
    }

    // Solve for control points using least squares
    GMlib::DMatrix<float> Nt = N;
    Nt.transpose();
    GMlib::DMatrix<float> NtN = Nt * N;
    GMlib::DMatrix<float> NtNi = NtN;
    NtNi.invert();
    GMlib::DMatrix<float> NtNiNt = NtNi * Nt;
    _controlPoints = NtNiNt * p;
}

// Evaluate basis function recursively using the Cox–de Boor formula
float MyB_spline::evaluateBasis(int i, int degree, float t) const {
    // Base case: degree 0 – check for the knot span; handle the special case when t equals the last knot.
    if( degree == 0 ) {
        if( (_knotVector[i] <= t && t < _knotVector[i+1]) ||
            (t == _knotVector[_knotVector.getDim()-1] && i == _controlPoints.getDim()-1) )
            return 1.0f;
        else
            return 0.0f;
    }
    
    float denom1 = _knotVector[i+degree] - _knotVector[i];
    float term1  = 0.0f;
    if( denom1 != 0.0f )
        term1 = (t - _knotVector[i]) / denom1 * evaluateBasis(i, degree - 1, t);
    
    float denom2 = _knotVector[i+degree+1] - _knotVector[i+1];
    float term2  = 0.0f;
    if( denom2 != 0.0f )
        term2 = (_knotVector[i+degree+1] - t) / denom2 * evaluateBasis(i+1, degree - 1, t);
    
    return term1 + term2;
}

// Evaluate the curve at parameter t using correct basis function evaluation
void MyB_spline::eval(float t, int d, bool left) const {
    this->_p.setDim(d+1);
    this->_p[0] = GMlib::Vector<float,3>(0.0f, 0.0f, 0.0f);
    
    int n = _controlPoints.getDim();
    int degree = 2; // B-spline degree
    
    // Sum over each control point multiplied by its corresponding basis function value.
    for (int i = 0; i < n; ++i) {
        float basisVal = evaluateBasis(i, degree, t);
        this->_p[0] += basisVal * _controlPoints[i];
    }
}

// Return start parameter
float MyB_spline::getStartP() const {
    return _knotVector[2]; // First non-repeated knot
}

// Return end parameter
float MyB_spline::getEndP() const {
    return _knotVector[_knotVector.getDim() - 3]; // Last non-repeated knot
}

// Just return false for now
bool MyB_spline::isClosed() const {
    return false;
}