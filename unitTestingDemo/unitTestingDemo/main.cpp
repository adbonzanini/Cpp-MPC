//
//  main.cpp
//  doubIntegratator
//
//  Created by Angelo Bonzanini on 12/11/20.
//

#define CATCH_CONFIG_RUNNER
#include <iostream>
#include "Tests.hpp"

int main(){
    
    std::cout << "This line gets executed before the tests" << std::endl;
    
    Catch::Session().run();
    std::cout << "===============================================================================\n";
    
    return 0;
}



