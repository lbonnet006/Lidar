#include <stdio.h>
#include <string.h>

#include "Lidar.hpp"

using namespace std;

//envoyer le port où est connecté le lidar, le nombre de points par scan ainsi que le nom du fichier où enregistrer
//Exemple: ./main /dev/ttyUSB0 1000 test
int main (int argc, char * argv []){
 
	Lidar *lidar;
    lidar = new Lidar(argv[1], atoi(argv[2]));
	
	lidar->connect();
	lidar->init(); 
	//lidar->calibration();
	lidar->scan_cart_file(argv[3]);
	//lidar->position();
	lidar->end();

    return 0;
}