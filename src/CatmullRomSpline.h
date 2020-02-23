/**
* /author Bret Jackson
*
* /file  CatmullRomSpline.H
* /brief Represents a piecewise 3rd-order Catmull-Rom spline based on the G3D10 implementation
*
*/ 

#ifndef CATMULLROMSPLINE_H
#define CATMULLROMSPLINE_H

#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <vector>
#include <corecrt_math.h>

//more troubleshooting from charlotte
//https://community.khronos.org/t/scaling-a-vec3/65095
template<typename Control>
class CatmullRomSpline
{
public:
	CatmullRomSpline() { 
		computeBasis();
	}
	
	~CatmullRomSpline() {}

	/** Appends a control point at a specific time that must be
        greater than that of the previous point. */
    void append(double t, const Control& c) {
        _time.push_back(t);
        _control.push_back(c);
    }

	/** Appends control point spaced in time based on the previous
        control point, or spaced at unit intervals if this is the
        first control point. */
    void append(const Control& c) {
        switch (_time.size()) {
        case 0:
            append(0, c);
            break;

        case 1:
            append(_time[0] + 1, c);
            break;

        default:
            append(2 * _time[_time.size() - 1] - _time[_time.size() - 2], c);
        }
    }

	void clear() {
        _control.clear();
        _time.clear();
    }


    /** Number of control points */
    int size() const {
        return (int)_control.size();
    }

	double totalTime() const {
		return _time[_time.size()-1];
	}

	void computeIndexInBounds(double s, int& i, double& u) const {
		int N = _time.size();
		double t0 = _time[0];
		double tn = _time[N - 1];
    
		i = std::floor((N - 1) * (s - t0) / (tn - t0));
    
		// Inclusive bounds for binary search
		int hi = N - 1;
		int lo = 0;
    
		while ((_time[i] > s) || (_time[i + 1] <= s)) {
        
			if (_time[i] > s) {
				// too big
				hi = i - 1;
			} else if (_time[i + 1] <= s) {
				// too small
				lo = i + 1;
			}
        
			i = (hi + lo) / 2;
		}
    
		// Having exited the above loop, i must be correct, so compute u.
		u = (s - _time[i]) / (_time[i + 1] - _time[i]);
	}

	void computeIndex(double s, int& i, double& u) const {
		int N = _time.size();
		assert(N > 0);
		double t0 = _time[0];
		double tn = _time[N - 1];
    
		if (N < 2) {
			// No control points to work with
			i = 0;
			u = 0.0;
		} else {
			// Non-cyclic
        
			if (s < t0) {
				// Non-cyclic, off the bottom.  Assume points are spaced
				// following the first time interval.
            
				double dt = _time[1] - t0;
				double x = (s - t0) / dt;
				i = std::floor(x);
				u = x - i;
            
			} else if (s >= tn) {
				// Non-cyclic, off the top.  Assume points are spaced following
				// the last time interval.
            
				double dt = tn - _time[N - 2];
				double x = N - 1 + (s - tn) / dt;
				i = std::floor(x);
				u = x - i;  
            
			} else {
				// In bounds, non-cyclic.  Assume a regular
				// distribution (which gives O(1) for uniform spacing)
				// and then binary search to handle the general case
				// efficiently.
				computeIndexInBounds(s, i, u);
            
			} // if in bounds
		} // extrapolation Mode
	}

	/** Returns a series of N control points and times, fixing
        boundary issues.  The indices may be assumed to be treated
        cyclically. */
    void getControls(int i, double* T, Control* A, int N) const {
        for (int j = 0; j < N; ++j) {
            getControl(i + j, T[j], A[j]);
        }
    }

	 /** Returns the requested control point and time sample based on
        array index.  If the array index is out of bounds linearly extrapolates (for a non-cyclic
        spline), assuming time intervals follow the first or last
        sample recorded.

        Returns 0 if there are no control points.
    */
    void getControl(int i, double& t, Control& c) const {
        int N = _control.size();
        if (N == 0) {
            t = 0;
        } else if (i < 0) {
            // Are there enough points to extrapolate?
            if (N >= 2) {
                // Step away from control point 0
                double dt = _time[1] - _time[0];
                
                // Extrapolate (note; i is negative)
				c = _control[1] * double(i) + _control[0] * double(1 - i);
                t = dt * i + _time[0];

            } else {
                // Just clamp
                c = _control[0];

                // Only 1 time; assume 1s intervals
                t = _time[0] + i;
            }

        } else if (i >= N) {
            if (N >= 2) {
                double dt = _time[N - 1] - _time[N - 2];

                // Extrapolate (note; i is negative)
                c = _control[N - 1] * double(i - N + 2) + _control[N - 2] * - double(i - N + 1);
                // Extrapolate
                t = _time[N - 1] + dt * (i - N + 1);

            } else {
                // Return the last, clamping
				c = _control[_control.size()-1];
                // Only 1 time; assume 1s intervals
                t = _time[0] + i;
            }
        } else {
            // In bounds
            c = _control[i];
            t = _time[i];
        }
    }

	void updateControl(int i, Control &c) {
		assert(i >=0 && i < _control.size());
		_control[i] = c;
	}

	void computeBasis() {
		// The standard Catmull-Rom spline basis (e.g., Watt & Watt p108)
		// is for [u^3 u^2 u^1 u^0] * B * [p[0] p[1] p[2] p[3]]^T.
		// We need a basis formed for:
		//
		//     U * C * [2*p'[1] p[1] p[2] 2*p'[2]]^T 
		//
		//     U * C * [p2-p0 p1 p2 p3-p1]^T 
		//
		// To make this transformation, compute the differences of columns in C:
		// For [p0 p1 p2 p3]
		//http://www.opengl-tutorial.org/beginners-tutorials/tutorial-3-matrices/
		//Adjusting things to make it all work actually well

		
		glm::dmat4 basis =
			glm::dmat4(-1, 2, -1, 0,
				3, -5, 0, 2,
				-3, 4, 1, 0,
				1, -1, 0, 0); //* glm::mat4(.5f);
			//*  0.5f;
		basis = basis.operator*=(.5f);
		// For [-p0 p1 p2 p3]^T 
		basis = glm::column(basis, 0, -glm::column(basis, 0));

		// For [-p0 p1 p2 p3-p1]^T 
		basis = glm::column(basis, 1, glm::column(basis, 1) + glm::column(basis, 3));

		// For [p2-p0 p1 p2 p3-p1]^T 
		basis = glm::column(basis, 2, glm::column(basis, 2) - glm::column(basis, 0));

		_basis = basis;
	}

	/**
       Return the position at time s.  The spline is defined outside
       of the time samples by extrapolation or cycling.
     */
    Control evaluate(double s) const {
        /*
        @cite http://www.gamedev.net/reference/articles/article1497.asp 
        Derivation of basis matrix follows.
        
        Given control points with positions p[i] at times t[i], 0 <= i <= 3, find the position 
        at time t[1] <= s <= t[2].

        Let u = s - t[0]
        Let U = [u^0 u^1 u^2 u^3] = [1 u u^2 u^3]
        Let dt0 = t[0] - t[-1]
        Let dt1 = t[1] - t[0]
        Let dt2 = t[2] - t[1]
         */

        // Index of the first control point (i.e., the u = 0 point)
        int i = 0;
        // Fractional part of the time
        double u = 0;

        computeIndex(s, i, u);

        Control p[4];
		double   t[4]; 
		
        getControls(i - 1, t, p, 4);

        const Control& p0 = p[0];
        const Control& p1 = p[1];
        const Control& p2 = p[2];
        const Control& p3 = p[3];

        // Compute the weighted sum of the neighboring control points.
        Control sum;

		/*
        if (interpolationMode == SplineInterpolationMode::LINEAR) {
            const float a = (s - t[1]) / (t[2] - t[1]);
            sum = p1 * (1.0f - a) + p2 * a;
            correct(sum);
            return sum;
        }
		*/

        double dt0 = t[1] - t[0];
        double dt1 = t[2] - t[1];
        double dt2 = t[3] - t[2];

        // Powers of u
        glm::dvec4 uvec((double)(u*u*u), (double)(u*u), (double)u, 1.0);

        // Compute the weights on each of the control points.
        const glm::dvec4& weights = uvec * _basis;
        

        // The factor of 1/2 from averaging two time intervals is 
        // already factored into the basis
        
        // tan1 = (dp0 / dt0 + dp1 / dt1) * ((dt0 + dt1) * 0.5);
        // The last term normalizes for unequal time intervals
        double x = (dt0 + dt1) * 0.5;
        double n0 = x / dt0;
        double n1 = x / dt1;
        double n2 = x / dt2;

        const Control& dp0 = (glm::dvec3)(p1 + p0) * -1.0;
        const Control& dp1 = (glm::dvec3)(p2 + p1) * -1.0;
        const Control& dp2 = (glm::dvec3)(p3 + p2) * -1.0;

        const Control& dp1n1 = (glm::dvec3)((glm::dvec3)dp1 *n1);
        const Control& tan1 = (glm::dvec3)((glm::dvec3)((glm::dvec3)dp0 * n0) + (glm::dvec3)dp1n1);
        const Control& tan2 = (glm::dvec3)((glm::dvec3)dp1n1 + (glm::dvec3)((glm::dvec3)dp2 * n2));

        sum = 
            (glm::dvec3)tan1 * weights[0] +
            (glm::dvec3)p1  * weights[1] +
            (glm::dvec3)p2  * weights[2] +
            (glm::dvec3)tan2 * weights[3]; 
        
		assert(isfinite(sum[0]) == true);
		assert(glm::isnan(sum[0]) != true);
        return sum;
    }

private:
	std::vector<double> _time;
	std::vector<Control> _control;
	glm::dmat4 _basis;
};

#endif