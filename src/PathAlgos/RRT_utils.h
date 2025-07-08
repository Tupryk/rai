#pragma once


namespace rai {

    void normalizeSphericalCoordinates(arr& x, const uintA& idx) {
      arr xsub = x({idx(0), idx(0)+idx(1)-1+1});
      op_normalize(xsub);
    }
  
    void randomSphericalCoordinates(arr& x, const uintA& idx) {
      arr xsub = x({idx(0), idx(0)+idx(1)-1+1});
      xsub = randn(xsub.N);
      op_normalize(xsub);
    }
  
    void flipSphericalCoordinates(arr& x, const uintA& idx) {
      arr xsub = x({idx(0), idx(0)+idx(1)-1+1});
      xsub *= -1.;
    }
  
    double corput(uint n, uint base) {
      double q = 0.;
      double bk = 1./double(base);
    
      while(n > 0) {
        q += (n % base)*bk;
        n /= base;
        bk /= double(base);
      }
      return q;
    }
  
    bool checkConnection(ConfigurationProblem& P,
                         const arr& start,
                         const arr& end,
                         const uint num,
                         const bool binary) {
      if(binary) {
        for(uint i=1; i<num; ++i) {
          double ind = corput(i, 2);
          arr p = start + ind * (end-start);
  
          // TODO: change to check feasibility properly (with path constraints)
          if(!P.query(p)->isFeasible) {
            return false;
          }
        }
      } else {
        for(uint i=1; i<num-1; ++i) {
          arr p = start + 1.0 * i / (num-1) * (end-start);
  
          // TODO: change to check feasibility properly (with path constraints)
          if(!P.query(p)->isFeasible) {
            return false;
          }
        }
      }
      return true;
    }
}

void revertPath(arr& path) {
  uint N = path.d0;
  arr x;
  for(uint i=0; i<N/2; i++) {
    x = path[i];
    path[i] = path[N-1-i];
    path[N-1-i] = x;
  }
}
