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
    
    std::cout << "EXECUTE TESTS..." << std::endl;
    Catch::Session().run();
    std::cout << "... DONE" << std::endl;

    
    return 0;
}



