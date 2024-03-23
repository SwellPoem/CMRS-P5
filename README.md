## Optimal Motion Planning for Localization of Avalanche Victims by Multiple UAVs

### Overview
This project aims to re-implement, extend, and test the methodologies presented in existing research for utilizing multiple drones(Unmanned Aerial Vehicles, UAVs) to find avalanche victims equipped with Avalanche Victim Detectors (ARTVA). 
The focus is on comparing two distinct operational approaches: a centralized system where decision-making is managed by a server with full information on all UAVs, and a decentralized system where drones operate based on partial information, without continuous communication with a central server or among themselves.

### Theoretical Framework
The project is built upon several key theoretical components:

- ARTVA Model: A mathematical representation of the ARTVA transceiver's functionality in both transmitting and receiving modes, essential for the drones' ability to detect and localize signals from avalanche victims.
- Estimation Algorithm: A critical piece of the project, this algorithm enables drones to estimate the position of the ARTVA signal based on the collected data.
- Exploration Strategies: Different strategies for drone movement and area scanning are tested, including linear, circular, and radial trajectories, to determine the most effective method for covering search areas.
- Average Consensus Filter (ACF): For the decentralized approach, the ACF is used to allow drones to form a consensus on the victim's location without centralized data processing.

### Simulation Details
Simulations were conducted in MATLAB, focusing on drone-ARTVA interactions, trajectory planning, and the effectiveness of centralized versus decentralized command structures under various scenarios. These simulations are designed to reflect realistic search conditions and assess the potential of UAV technology in avalanche search and rescue operations.

### Experiments and Findings
The project conducts extensive testing on:

- The impact of different numbers of drones and their assigned trajectories on the efficiency and success of the search operation.
The comparative analysis of centralized and decentralized systems in terms of information management, decision-making accuracy, and operational flexibility.
- The effectiveness of exploration strategies in maximizing area coverage and minimizing the time to locate the ARTVA signal.
- The findings highlight the strengths and limitations of each approach, offering insights into optimizing UAV use in search and rescue missions.

### References
- Camilla Tabasso, Nicola Mimmo, Venanzio Cichella, and Lorenzo Marconi. Optimal motion planning for localization of avalanche victims by multiple uavs. IEEE Control Systems Letters, 5(6):2054–2059, 2021. doi: 10.1109/LCSYS.2021.3049314.
- Nicola Mimmo, Pauline Bernard, and Lorenzo Marconi. Avalanche victim search via robust observers. IEEE Transactions on Control Systems Technology, 29(4):1450–1461, 2021. doi:10.1109/TCST.2020.3016665.
