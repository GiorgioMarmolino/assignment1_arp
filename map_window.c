#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <errno.h>
#include "helper.h"

FILE *debug, *errors;           // File descriptors for the two log files
Game game;
Drone *drone;
int server_write_fd;            // File descriptor for sending the size of the map to the server
Object obstacles, targets;
int n_obs;
int n_targ;

void draw_outer_box() {
    attron(COLOR_PAIR(1));
    box(stdscr, 0, 0);
    mvprintw(0, 1, "Dimension of the window: %d x %d", game.max_x, game.max_y);
    attroff(COLOR_PAIR(1));
    refresh();
}

void render_obstacles(Object obstacles[]) {
    attron(COLOR_PAIR(3));
    for(int i = 0; i < n_obs; i++){
        mvprintw(obstacles[i].pos_y, obstacles[i].pos_x, "#");
    }
    attroff(COLOR_PAIR(3));
    refresh();
}

void render_targets(Object targets[]) {
    attron(COLOR_PAIR(3));
    for(int i = 0; i < n_targ; i++){
        mvprintw(targets[i].pos_y, targets[i].pos_x, "#");
    }
    attroff(COLOR_PAIR(3));
    refresh();
}

void render_drone(float x, float y) {
    attron(COLOR_PAIR(4));
    mvprintw(y, x, "+");
    attroff(COLOR_PAIR(4));
    refresh();
}

void write_to_server() {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "%d, %d", game.max_x, game.max_y);
    write(server_write_fd, buffer, strlen(buffer));
}

// Resize the input window
void resize_window() {
    endwin();
    refresh();
    clear();

    getmaxyx(stdscr, game.max_y, game.max_x);
    resize_term(game.max_y, game.max_x);

    write_to_server();

    clear();
    refresh();

    draw_outer_box();
}

// Handler for the signal SIGWINCH
void resize_handler(int sig, siginfo_t *info, void *context) {
    if (sig == SIGWINCH) {
        resize_window();
    }
    
    if (sig == SIGUSR2){
        LOG_TO_FILE(debug, "Shutting down by the SERVER");
        // Close the files
        fclose(errors);
        fclose(debug);
        endwin();
        exit(EXIT_SUCCESS);
    }
}

int open_shared_memory() {
    int mem_fd = shm_open(DRONE_SHARED_MEMORY, O_RDONLY, 0666);
    if (mem_fd == -1) {
        perror("Error opening the shared memory");
        LOG_TO_FILE(errors, "Error opening the shared memory");
        // Close the files
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    drone = (Drone *)mmap(0, sizeof(Drone), PROT_READ, MAP_SHARED, mem_fd, 0);
    if (drone == MAP_FAILED) {
        perror("Error mapping the shared memory");
        LOG_TO_FILE(errors, "Error mapping the shared memory");
        // Close the files
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    return mem_fd;
}

void map_render(Drone *drone) {
    char buffer[256];
    fd_set read_fds;
    struct timeval timeout;
    while(1){
        clear();
        draw_outer_box();
        render_drone(drone->pos_x, drone->pos_y);
        usleep(50000);
    }

    /*int max_fd = -1;
    if (... > max_fd) {
        max_fd = ...;
    }

    while(1) {
        FD_ZERO(&read_fds);
        FD_SET(..., &read_fds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;
        int activity;
        do {
            activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        } while(activity == -1 && errno == EINTR);

        if (activity < 0) {
            perror("Error in the map's select");
            LOG_TO_FILE(errors, "Error in select which pipe reads");
            break;
        } else if (activity > 0) {
            // Check if the map process has sent him the map size
            if (FD_ISSET(..., &read_fds)) {
                ssize_t bytes_read = read(..., buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0'; // End the string
                    sscanf(buffer, "%d, %d", &game.max_x, &game.max_y);
                }
            }
        } else {
            clear();
            draw_outer_box();
            render_drone(drone->pos_x, drone->pos_y);
        }
    }*/
}

int main(int argc, char *argv[]) {
    

    /* OPEN THE LOG FILES */
    debug = fopen("debug.log", "a");
    if (debug == NULL) {
        perror("Error opening the debug file");
        exit(EXIT_FAILURE);
    }
    errors = fopen("errors.log", "a");
    if (errors == NULL) {
        perror("Error opening the errors file");
        exit(EXIT_FAILURE);
    }

    if (argc < 5) {
        LOG_TO_FILE(errors, "Invalid number of parameters");
        // Close the files
        fclose(debug);
        fclose(errors); 
        exit(EXIT_FAILURE);
    }

    LOG_TO_FILE(debug, "Process started");

    /* SETUP THE PIPE */
    server_write_fd = atoi(argv[1]);
    int server_read_fd = atoi(argv[2]);
    n_obs = atoi(argv[3]);
    n_targ = atoi(argv[4]);

    LOG_TO_FILE(errors, argv[2]);



    /* SETUP NCURSE */
    initscr(); 
    start_color();
    cbreak(); 
    noecho();
    curs_set(0);

    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_RED, COLOR_BLACK);
    init_pair(4, COLOR_CYAN, COLOR_BLACK);

    /* SETUP SIGNALS */
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = resize_handler;
    sigemptyset(&sa.sa_mask);
    // Set the signal handler for SIGWINCH
    if (sigaction(SIGWINCH, &sa, NULL) == -1) {
        perror("Error in sigaction(SIGWINCH)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGWINCH)");
        // Close the files
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    // Set the signal handler for SIGUSR2
    if(sigaction(SIGUSR2, &sa, NULL) == -1){
        perror("Error in sigaction(SIGURS2)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGURS2)");
        // Close the files
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }

    /* OPEN THE SHARED MEMORY */
    int mem_fd = open_shared_memory();

    // Retrive the dimension of the terminal
    getmaxyx(stdscr, game.max_y, game.max_x);
    // Send to the server the dimension
    write_to_server();

    char buffer[256];
    fd_set read_fds;
    struct timeval timeout;

    int max_fd = -1;
    if (server_read_fd > max_fd) {
        max_fd = server_read_fd;
    }

    /* LAUNCH THE MAP */
    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(server_read_fd, &read_fds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int activity;
        do {
            activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        } while(activity == -1 && errno == EINTR);

        if (activity < 0) {
            perror("Error in the server's select");
            LOG_TO_FILE(errors, "Error in select which pipe reads");
            break;
        } else if (activity > 0) {
            // Check if the map process has sent him the map size
            if (FD_ISSET(server_read_fd, &read_fds)) {
                LOG_TO_FILE(errors, "Entrato billy");
                ssize_t bytes_read = read(server_read_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0'; // End the string
                    //sscanf(buffer, "%d, %d", &game.max_x, &game.max_y);
                    LOG_TO_FILE(errors, buffer);
                }
            }
        } else {
            map_render(drone);
        }
    }    
    //map_render(drone);

    /* END PROGRAM*/
    endwin();
    // Close the file descriptor
    if (close(mem_fd) == -1) {
        perror("Close file descriptor");
        LOG_TO_FILE(errors, "Error in closing the shared memory");
        // Close the files
        fclose(debug);
        fclose(errors); 
        exit(EXIT_FAILURE);
    }
    // Unmap the shared memory region
    munmap(drone, sizeof(Drone));

    // Close the files
    fclose(debug);
    fclose(errors);

    return 0;
}
