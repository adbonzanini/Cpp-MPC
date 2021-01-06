//
//  main.cpp
//  doubIntegratator
//
//  Created by Angelo Bonzanini on 12/11/20.
//

#define CATCH_CONFIG_RUNNER
#include <iostream>
#include <math.h>
#include <chrono>
#include <random>
#include "pbPlots.hpp"
#include "supportLib.hpp"
#include "Tests.hpp"



int main(int argc, const char * argv[]) {
    
    
    // Create Parameters object and define initial state
    Params p;
    Eigen::VectorXd x0(p.nx); x0(0) = 4; x0(1) = 5;
    
    auto start = std::chrono::high_resolution_clock::now();
    MPC::ReturnTraj tr = solveMPC(p,x0);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    
    
    
    
    
    
    
    // ############ PRINTING AND PLOTTING (can comment it out) ############
    // Print solution
    std::cout << "x1 = ";
    for(int j=0;j<p.Nsim+1;j++){std::cout << "[" << j << "]:" << tr.x1Path[j] << " ";}
    std::cout << std::endl;
    std::cout << "x2 = ";
    for(int j=0;j<p.Nsim+1;j++){std::cout << "[" << j << "]:" << tr.x2Path[j] << " ";}
    std::cout << std::endl;
    std::cout << "u1 = ";
    for(int j=0;j<p.Nsim;j++){std::cout << "[" << j << "]:" << tr.u1Path[j] << " ";}
    std::cout << std::endl;
    // To add a value: xPlot.push_back(value)
    
    
    start = std::chrono::high_resolution_clock::now();
    // Create discrete time vectors for plotting
    std::vector<double> discreteTime(p.Nsim+1);
    std::iota(discreteTime.begin(), discreteTime.end(), 0);
    std::vector<double> discreteTimeU = std::vector<double> (discreteTime.begin(), discreteTime.end()-1);
    // Print last element
    // std::cout << discreteTime.back() << " " << discreteTimeU.back() << std::endl;
    
    
    // Plotting and saving
    RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();
    std::string imgLoc = "/Users/adbonzanini/Box Sync/Berkeley/Research/Cpp-MPC/doubleIntegratorMPCclasses/doubleIntegratorMPCclasses/";
    std::string imgTitle;
    std::vector<double> *pngData;
    
    // Plot 1
    imgTitle = "phasePlot.png";
    DrawScatterPlot(imageRef, 600, 400, &tr.x1Path, &tr.x2Path);
    pngData = ConvertToPNG(imageRef->image);
    WriteToFile(pngData, imgLoc+imgTitle);
    
    // Plot 2
    imgTitle = "x1.png";
    DrawScatterPlot(imageRef, 600, 400, &discreteTime, &tr.x1Path);
    pngData = ConvertToPNG(imageRef->image);
    WriteToFile(pngData, imgLoc+imgTitle);
    
    // Plot 3
    imgTitle = "x2.png";
    DrawScatterPlot(imageRef, 600, 400, &discreteTime, &tr.x2Path);
    pngData = ConvertToPNG(imageRef->image);
    WriteToFile(pngData, imgLoc+imgTitle);
    
    DeleteImage(imageRef->image);
    
    stop = std::chrono::high_resolution_clock::now();
    std::cout << std::endl;
    std::cout << "Avg. time per iteration = " <<duration.count()/p.Nsim << " ms" << std::endl;
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time for plot and save  = " << duration.count() << " ms"  << std::endl;

    
    
    
    // ############ RUN TESTS ############
    std::cout << "\nEXECUTING TESTS..." << std::endl;
    Catch::Session().run();
    std::cout << "... DONE\n" << std::endl;
    
    
    // For reference
    /*
    // Indexing
    std::cout << tr.x1Path[tr.x1Path.size()-1] << std::endl;
    
    // Slicing
    std::vector<double> v2 = std::vector<double> (tr.x1Path.begin()+15, tr.x1Path.end());
    std::cout << v2[0] << std::endl;
     */

    return 0;
}
