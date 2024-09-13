# ros2_ignition_thesis
ROS 2 Humble + Ignition Fortress
Questo repository contiene 4 package ROS 2 e il pdf della Tesi di laurea.
### Descrizione contenuto package
1. my_sim_tesi_bringup
  * config/ -> contiene i file di configurazione dei nodi (file .yaml) e di rviz2 (file .rviz);
  * files/ -> contiene il file coordinates_goal.txt che viene passato come parametro alla classe PoseControlActionClient per definire le posizioni obiettivo che devono essere di volta in volta raggiunte;
  * launch/ -> contiene i file di lancio Python:
    1. Per avviare la scansione della mappa statica con Slam Toolbox utilizzare my_sim_map_scan.launch.py
    2. Per avviare la simulazione utilizzare my_sim_final.launch.py (che al suo interno lancia anche i file my_ros2_nodes.launch.py e nav2.launch.py)
  * map/ -> contiene i file delle mappe (my_map.yaml e my_map.pgm) generati al termine della scansione della mappa statica (launch file my_sim_map_scan.launch.py) con il comando da terminale Linux "ros2 run nav2_map_server map_saver_cli"
2. my_sim_tesi_gazebo
  * models/ -> è la cartella contenente tutti i modelli sdf utilizzati nella simulazione
  * worlds/ -> è la cartella contenente gli worlds utilizzati nella simulazione (in questo caso uno solo, my_world.sdf) 
3. my_sim_tesi_ros2_interfaces
  * action/ -> contiene il file che definisce la sola action utilizzata nella simulazione (file DronePoseControl.action per la action DronePoseControl)
4. my_sim_tesi_ros2_nodes
  * include/my_sim_tesi_ros2_nodes/ -> contiene i file header hpp:
    1. orchestrator_node.hpp
    2. pose_control_action_client.hpp
    3. pose_control_action_server.hpp
  * src -> contiene i file C++ che implementano gli header (presenti nella cartella include/my_sim_tesi_ros2_nodes/):
    1. orchestrator_node.cpp
    2. pose_control_action_client.cpp
    3. pose_control_action_server.cpp
