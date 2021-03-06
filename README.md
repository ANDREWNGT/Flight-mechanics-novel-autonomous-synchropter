# Flight-mechanics-novel-autonomous-synchropter
6 week group design project of a novel, autonomous synchropter design, meant to retrieve a payload undergoing reentry. Worked in the flight mechanics and controls team. An individual report is included. Project was completed in fulfilment of the Masters of Engineering (MEng Degree) in Aeronautical Engineering, Imperial College London. 

Further areas to explore included: Machine learning to aid controls, inspired by the Stanford Autonomous Helicopter group: http://heli.stanford.edu/

An individual report detailing the flight mechanics considerations required in producing this novel synchropter design, along with the alignment to mission objectives for which the rotorcraft is designed for. A state space model with 13 variables was produced. Rotor flap and higher order harmonic frequencies were ignored. A slung load was considered an included within the state space model. 

Folder "src" contains all source code for computation. Note: MATLAB 2017 and above is required. As this project was conceptual, code optimisation was not a priority; many variables were tinkered with and time and space complexity analysis were not accounted for. 

To run the code, simply run the script "main_script.m". All other processes are automated and the output should be in the "../mission_legs/state_space_model.mat" file. 

