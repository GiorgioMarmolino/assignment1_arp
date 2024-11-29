#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#define MASS 5    
#define FRICTION_COEFFICIENT 0.5   
#define FORCE_MODULE 1.0 
#define T 0.5
#define MAXFREP 5 

typedef struct { 
    int x;
    int y;
    float vx;
    float vy;
    float fx, fy;
} Drone; 

typedef struct { 
    int x;
    int y;
} Obstacle; 

float rho0 = 8; 
float rho1 = 0.5;
float rho2 = 2;
float eta = 40; 

bool collsion[5];

/*void draw_target(WINDOW *win, Position target, int number) {
    wattron(win, COLOR_PAIR(2));
    mvwaddch(win, target.y, target.x, '1' + number);
    wattroff(win, COLOR_PAIR(2));
}*/

/*void draw_obstacle(WINDOW *win, Obstacle obstacle) {
    wattron(win, COLOR_PAIR(3));
    mvwaddch(win, obstacle.y, obstacle.x, '#');
    wattroff(win, COLOR_PAIR(3));
}*/

Obstacle random_position(int max_y, int max_x) {
    Obstacle pos;
    pos.y = rand() % max_y;
    pos.x = rand() % max_x;
    return pos;
}

float calculateFrictionForce(float velocity) {
    return -FRICTION_COEFFICIENT * (velocity);
}

float calculateRepulsiveForcex(Drone dr, int xo, int yo){

    float rho = sqrt(pow(dr.x - xo, 2) + pow(dr.y - yo, 2));
    if (rho < 0.5) rho = 0.5; // Imposta un valore minimo per rho
    float theta = atan2(dr.y - yo, dr.x - xo);
    float fx;

    if (rho < rho0) {
        fx = eta * (1 / rho - 1 / rho0) * cos(theta) * fabs(dr.vx); // Rimosso 1/rho^2
    } else {
        fx = 0;
    }

    // Clamp della forza
    if (fx > MAXFREP) fx = MAXFREP;
    if (fx < -MAXFREP) fx = -MAXFREP;

    return fx;
}

float calculateRepulsiveForcey(Drone dr, int xo, int yo) {
    float rho = sqrt(pow(dr.x - xo, 2) + pow(dr.y - yo, 2));
    if (rho < 0.5) rho = 0.5; // Imposta un valore minimo per rho
    float theta = atan2(dr.y - yo, dr.x - xo);
    float fy;

    if (rho < rho0) {
        fy = eta * (1 / rho - 1 / rho0) * sin(theta) * fabs(dr.vy); // Rimosso 1/rho^2
    } else {
        fy = 0;
    }

    // Clamp della forza
    if (fy > MAXFREP) fy = MAXFREP;
    if (fy < -MAXFREP) fy = -MAXFREP;

    return fy;
}


void update_drone(Drone *dr, int max_x, int max_y, float dt, Obstacle *edges[], int nedges) {
    
    float fx_edge = 0;
    float fy_edge = 0;

    float frictionForceX = calculateFrictionForce(dr->vx);
    float frictionForceY = calculateFrictionForce(dr->vy);

    for(int i = 0; i < nedges; i++){
            fx_edge += calculateRepulsiveForcex(*dr, edges[i]->x, edges[i]->y);
            fy_edge += calculateRepulsiveForcey(*dr, edges[i]->x, edges[i]->y);
        }

    float accelerationX = (dr->fx + frictionForceX + fx_edge) / MASS;
    float accelerationY = (dr->fy + frictionForceY + fy_edge) / MASS;

    dr->vx += accelerationX * dt;
    dr->vy += accelerationY * dt;
    dr->x += dr->vx * dt + 0.5 * accelerationX * dt * dt;
    dr->y += dr->vy * dt + 0.5 * accelerationY * dt * dt;

    /*if (dr->x < 0) { dr->x = 0; dr->vx = 0; }
    if (dr->x >= max_x) { dr->x = max_x - 1; dr->vx = 0; }
    if (dr->y < 0) {dr->y = 0; dr->vy = 0; }
    if (dr->y >= max_y) { dr->y = max_y - 1; dr->vy = 0; }*/

}


void handle_input(int ch, Drone *dr) {
    switch (ch) {
        case 'w': dr->fy -= 1.0; break;
        //case 'e': dr->fy -= sqrt(2)/2; dr->fx += sqrt(2)/2; break; 
        case 'd': dr->fx += 1.0; break; 
        //case 'x': dr->fy += sqrt(2)/2; dr->fx += sqrt(2)/2; break; 
        case 's': dr->fy += 1.0; break; 
        //case 'z': dr->fy += sqrt(2)/2; dr->fx -= sqrt(2)/2;break; 
        case 'a': dr->fx -= 1.0; break; 
        //case 'q': dr->fy -= sqrt(2)/2; dr->fx -= sqrt(2)/2; break; 
        
        case ' ': dr->vx = 0.0; dr->vy = 0.0; dr->fx = 0.0; dr->fy = 0.0; break; // Ferma tutto
    }
}

void render_drone(Drone dr) {
    mvprintw(dr.y, dr.x, "+");
}

void render_obstacles(Obstacle obs[]) {
    for(int i=0; i<5; i++){
        mvprintw(obs[i].y, obs[i].x, "#");
    }
}

int main(int argc, char* argv[]){

    int max_y, max_x;

    initscr();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    curs_set(0);

    getmaxyx(stdscr, max_y, max_x);

    int nedges = 2 * (max_x + max_y);  // 2 bordi orizzontali + 2 bordi verticali
    Obstacle *edges[nedges];

    // Alloca memoria per gli ostacoli sui bordi
    for (int i = 0; i < max_y; i++) {
        edges[i] = malloc(sizeof(Obstacle));
        edges[i]->x = 0;
        edges[i]->y = i;
        //mvprintw(edges[i]->y, edges[i]->x, "|");

        edges[i + max_y + max_x] = malloc(sizeof(Obstacle));  // Bordi destra
        edges[i + max_y + max_x]->x = max_x;
        edges[i + max_y + max_x]->y = i;
        //mvprintw(edges[i + max_y + max_x]->y, edges[i]->x, "|");
    }

    for (int i = 0; i < max_x; i++) {
        edges[i + max_y] = malloc(sizeof(Obstacle)); // Bordi superiori
        edges[i + max_y]->x = i;
        edges[i + max_y]->y = max_y;
        //mvprintw(edges[i + max_y]->y, edges[i]->x, "|");

        edges[i + 2 * max_y + max_x] = malloc(sizeof(Obstacle)); // Bordi inferiori
        edges[i + 2 * max_y + max_x]->x = i;
        edges[i + 2 * max_y + max_x]->y = 0;
        //mvprintw(edges[i + 2 * max_y + max_x]->y, edges[i]->x, "|");
    }



    int x0 = max_x/2;  
    int y0 = max_y/2;

    //Obstacle target = random_position(max_y, max_x);

    Obstacle obstacle[5];
    for(int i=0; i<5; i++){
        obstacle[i] = random_position(max_y, max_x);
        collsion[i] = false;
    }
      

    Drone dr;

    float vx0 = 0, vy0 = 0;

    int fx0 = 0, fy0 = 0;

    dr.x = x0;
    dr.y = y0;
    dr.vx = vx0;
    dr.vy = vy0;
    dr.fx = fx0;
    dr.fy = fy0;

    while(1){
        clear();

        render_obstacles(obstacle);
        render_drone(dr);

        timeout(50);
        int ch = getch();
        if (ch == 'm') break; 
        if (ch != ERR) handle_input(ch, &dr);

        /*for(int i = 0; i < nedges; i++){
            fx_edge += calculateRepulsiveForcex(dr, edges[i]->x, edges[i]->y);
            fy_edge += calculateRepulsiveForcey(dr, edges[i]->x, edges[i]->y);
        }
        */

        update_drone(&dr, max_x, max_y, T, edges, nedges);

        refresh();
        usleep(50000); 
    }

    endwin();
    return 0;

}