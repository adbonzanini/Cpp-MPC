//
//  FG_eval.hpp
//  doubleIntegrator
//
//  Created by Angelo Bonzanini on 12/11/20.
//

#ifndef FG_eval_hpp
#define FG_eval_hpp

#include "Params.hpp"


class FG_eval{
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
//    ADvector fg, vars;
    void operator()(ADvector & fg, const ADvector & vars);
};

#endif /* FG_eval_hpp */
