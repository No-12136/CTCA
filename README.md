# Stochastic CTCA model
Simulation Algorithm for Stochastic CTCA (Continuous Traffic Cellular Automata) Model with adjustable boundary conditions (Open boundary or Periodic boundary), multi-lane, multi-vehicle type, and including axle information.  
Note that CTCA is a class of CA model for traffic flow simulation, but differs significantly from continuous CA models in CA models. Please refer to the article published by Öznur Yeldan et al. in 2012 (Yeldan, Ö., Colorni, A., Luè, A., & Rodaro, E. (2012). A Stochastic Continuous Cellular Automata Traffic Flow Model with a Multi-agent Fuzzy System. Procedia - Social and Behavioral Sciences, 54, 1350–1359. https://doi.org/10.1016/j.sbspro.2012.09.849) for the detailed principles. Unlike the general TCA (Traffic CA) model, the CTCA model treats vehicles as cells, which ensures spatial continuity and fundamentally solves the problem that vehicle position, velocity and acceleration cannot be taken continuously. Compared with the TCA model, this model is closer to the CF (Car-Following) model, while retaining the features and advantages of the TCA model.  
In the given algorithm, the open boundary defaults to a Poisson distribution-based departure model, the local transition rule uses the VE (velocity effect) model with anticipation (Please refer to the article [1] Lárraga, M. E., Río, J. A. del, & Schadschneider, A. (2004). New kind of phase separation in a CA traffic model with anticipation. Journal of Physics A: Mathematical and General, 37(12), 3769. https://doi.org/10.1088/0305-4470/37/12/004 and [2] Li, X., Wu, Q., & Jiang, R. (2001). Cellular automaton model considering the velocity effect of a car on the successive car. Physical Review E, 64(6), 066128), the vehicle acceleration is processed with continuity, and the lane-changing rule uses an asymmetric lane-changing rule for a typical three lanes (overtaking, travelling, and slow lanes).  
The algorithm has some adjustable model parameters or traffic flow statistics parameters at the beginning of the main function in addition to the input parameters, so please adjust them according to your needs.  
The main function of the algorithm is Sto_CTCA.m and the rest are sub-functions, some of which are implemented as classes to distinguish different boundary conditions. The algorithm itself does not contain any output, but only generates a dynamic graph of the evolution of the traffic flow in the form of a gif at the specified file location (implemented as a subfunction Traffic_map.m).  
[![View Stochastic CTCA model on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://ww2.mathworks.cn/matlabcentral/fileexchange/156542-stochastic-ctca-model)
