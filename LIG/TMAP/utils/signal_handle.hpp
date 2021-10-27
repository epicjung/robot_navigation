#pragma once

#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>

namespace signal_handle
{
    void signal_callback_handler(int signum) {
        std::cout << "\033[34;1m" << "Caught signal "<<  signum << "\033[32;0m" <<std::endl;
        // Terminate program
        exit(signum);
    }
} // namespace signal_handle

// Define the function to be called when ctrl-c (SIGINT) is sent to process

// // Example for usage
// int main(){
//    // Register signal and signal handler
//    signal(SIGINT, signal_handle::signal_callback_handler);
//    while(true){
//       cout << "Program processing..." << endl;
//       sleep(1);
//    }
//    return EXIT_SUCCESS;
// }
