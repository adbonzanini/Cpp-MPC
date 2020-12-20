//
//  Tests.cpp
//  unitTestingDemo
//
//  Created by Angelo Bonzanini on 12/20/20.
//

#include "Tests.hpp"

unsigned int Factorial( unsigned int number ) {
    return number > 1 ? Factorial(number-1)*number : 1;
}


TEST_CASE("The ultimate answer"){
    REQUIRE( 6*7 == 42);
}

TEST_CASE("10x10 ints"){
    auto x = GENERATE(range(1, 11));
    auto y = GENERATE(range(101, 111));
    
    CHECK(x<y);
}

TEST_CASE("Random numbers in a range", "[.] [approvals]"){
    auto x = GENERATE( -1000+rand()%10000+1 );
    auto y = GENERATE(random(-1000, 1000));
    CAPTURE(x);
    REQUIRE(  x*x>= 0);
    REQUIRE(  y*y>= 0);
}

TEST_CASE( "Factorials are computed", "[factorial]" ) {
    CHECK( Factorial(0) == 1 );
    CHECK( Factorial(1) == 1 );
    CHECK( Factorial(2) == 2 );
    CHECK( Factorial(3) == 6 );
    CHECK( Factorial(10) == 3628800 );
}

TEST_CASE("Vectors can be sized and resized", "[vector]"){
    std::vector<int> v(5);
    
    REQUIRE(v.size()==5);
    REQUIRE(v.capacity()>=5);
    
    // Each section executes the test case from the start!
    SECTION("Resizing bigger changes size and capacity"){
        v.resize(10);
        
        REQUIRE(v.size()==10);
        REQUIRE(v.capacity()>=10);
    }
    
    SECTION("Resizing smaller changes size but not capacity"){
        v.resize(1);
        
        REQUIRE(v.size()==1);
        REQUIRE(v.capacity()>=5);
    }
    
    SECTION("reserving bigger changes capacity but not size"){
        v.reserve(10);
        
        REQUIRE(v.size()==5);
        REQUIRE(v.capacity()>=10);
        
        SECTION("reserving smaller again does not change capacity"){
            v.reserve(7);
            REQUIRE(v.capacity()>=10);
        }
    }
        
    
    SECTION("reserving smaller does not change capacity or size"){
        v.reserve(1);
        
        REQUIRE(v.size()==5);
        REQUIRE(v.capacity()>=5);
    }
    
    
}
