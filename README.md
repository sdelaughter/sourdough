#CEFGW: A PID Controller for Stabilizing Delay while Maximizing Thorughput on Cellular Networks with Unpredictable and Highly Variable Link Capacity

##by Sam DeLaughter and Brandon Karpf

###Written for the Fall 2016 section of course 6.829 at MIT

####The only files modified from the original sourdough version are controller.cc and this README

####To run
cd to the sourdough directory and enter:

    ./run

This command will recompile the code before running in case any changes have been made.


####Code for exercises A, B, and C can be viewed in files controllerA.cc, controllerB.cc, and controllerC.cc

controllerA.cc requires manual modification to adjust the fixed window size.

controllerB.cc and controllerC.cc will not run, as they require slight modifications to controller.hh and sender.cc in order to handle packet loss events.



####Please see writeup.pdf for a detailed explanation of our controller.
