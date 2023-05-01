#ifndef LIDAR_H_
#define LIDAR_H_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <paths.h>
#include <termios.h>
#include <sysexits.h>
#include <sys/param.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <cassert>
#include <iostream>
#include <cstdint>
#include <unistd.h>
#include <math.h>
#include <sys/ioctl.h> //ioctl() call defenitions

using namespace std;

class Lidar {
    private:
        int fileID = -1;
        string port = "/dev/ttyUSB0";

        int nb_point = 1000;
        int nb_calibration = 3;

        int dist_min = 150;
	    int dist_max = 3800;

        float dist_seuil = 100.;
        float taille_rep_min = 60.;
        float taille_rep_max = 180.;
        int nbr_seuil = 4;
        int nbr_seuil_max = 200;
        float seuil_saut = 100.;
        float seuil_balise = 100.;

        float d_min_1 = 2300;
        float d_max_1 = 2700;
        float d_min_2 = 2600;
        float d_max_2 = 3200;

        float x1 = 0;
        float y1 = 0;
        float x2 = 0;
        float y2 = 3000;

        int nb_part = 0;

        float balises_X[3];
        float balises_Y[3];
        float angle_balises[3];

        float x;
        float y;

        bool affichage = true;

    public:
        Lidar(string p, int n);

        void write_int(const uint8_t value);
        uint8_t read_uint8();
        int read_uint8_n(uint8_t *value, int n);


        void set_port(string p);
        void set_threshold(int min, int max);
        void set_nb_point(int n);

        int connect();
        void end();
        int init();
        int reset();
        int info();
        void calibration();
        int scan_pol(float *data, float *angle);   
        int scan_pol_file(string file);    
        int scan_cart(float *X, float *Y);
        int scan_cart_file(string file);    
        int scan_img(string file);

        void tri(float *data, float *angle, float *data_tri, float *angle_tri);
        void point_part(float *data, float *angle, float *X_part, float *Y_part, float *angle_part);
        int balises(float *X_part, float *Y_part, float *angle_part);
        int position();

        void get_position(int *pos_x, int *pos_y);
};

#endif