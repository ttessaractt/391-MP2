/*
 * tab:4
 *
 * mazegame.c - main source file for ECE398SSL maze game (F04 MP2)
 *
 * "Copyright (c) 2004 by Steven S. Lumetta."
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice and the following
 * two paragraphs appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE AUTHOR OR THE UNIVERSITY OF ILLINOIS BE LIABLE TO 
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES ARISING OUT  OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, 
 * EVEN IF THE AUTHOR AND/OR THE UNIVERSITY OF ILLINOIS HAS BEEN ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND THE UNIVERSITY OF ILLINOIS SPECIFICALLY DISCLAIM ANY 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE 
 * PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND NEITHER THE AUTHOR NOR
 * THE UNIVERSITY OF ILLINOIS HAS ANY OBLIGATION TO PROVIDE MAINTENANCE, 
 * SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Author:        Steve Lumetta
 * Version:       1
 * Creation Date: Fri Sep 10 09:57:54 2004
 * Filename:      mazegame.c
 * History:
 *    SL    1    Fri Sep 10 09:57:54 2004
 *        First written.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>

#include "blocks.h"
#include "maze.h"
#include "modex.h"
#include "text.h"

// New Includes and Defines
#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/io.h>
#include <termios.h>
#include <pthread.h>

#include "module/tuxctl-ioctl.h"

#define BACKQUOTE 96
#define UP        65
#define DOWN      66
#define RIGHT     67
#define LEFT      68


/*
    up = ef
    down = df
    left = bf
    right = 7f
    start = fe
*/
#define T_UP        0xEF
#define T_DOWN      0xDF
#define T_RIGHT     0x7F
#define T_LEFT      0xBF
#define T_START     0xFE
/*
 * If NDEBUG is not defined, we execute sanity checks to make sure that
 * changes to enumerations, bit maps, etc., have been made consistently.
 */
#if defined(NDEBUG)
#define sanity_check() 0
#else
static int sanity_check();
#endif


/* a few constants */
#define PAN_BORDER      5  /* pan when border in maze squares reaches 5    */
#define MAX_LEVEL       10 /* maximum level number                         */

/* outcome of each level, and of the game as a whole */
typedef enum {GAME_WON, GAME_LOST, GAME_QUIT} game_condition_t;

/* structure used to hold game information */
typedef struct {
    /* parameters varying by level   */
    int number;                  /* starts at 1...                   */
    int maze_x_dim, maze_y_dim;  /* min to max, in steps of 2        */
    int initial_fruit_count;     /* 1 to 6, in steps of 1/2          */
    int time_to_first_fruit;     /* 300 to 120, in steps of -30      */
    int time_between_fruits;     /* 300 to 60, in steps of -30       */
    int tick_usec;         /* 20000 to 5000, in steps of -1750 */
    
    /* dynamic values within a level -- you may want to add more... */
    unsigned int map_x, map_y;   /* current upper left display pixel */
} game_info_t;

static game_info_t game_info;

/* local functions--see function headers for details */
static int prepare_maze_level(int level);
static void move_up(int* ypos);
static void move_right(int* xpos);
static void move_down(int* ypos);
static void move_left(int* xpos);
static int unveil_around_player(int play_x, int play_y);
static void *rtc_thread(void *arg);
static void *keyboard_thread(void *arg);
static void *tux_thread(void *arg);
static void tux_timer(int min, int sec);

//function to combine text in status bar, update text, and create buffer from text
static void status_bar(unsigned char sb_buf[], unsigned int level, unsigned int minutes, unsigned int seconds);

//colors for changing maze walls with level, 1 palette per level
static unsigned char wall_colors[10][3] = {
    /*{ 0x9E, 0x01, 0x42 },{ 0xD5, 0x3E, 0x4F }, 
    { 0xF4, 0x6D, 0x43 },{ 0xFD, 0xAE, 0x61 },
    { 0xFE, 0xE0, 0x8B },{ 0xE6, 0xF5, 0x98 },
    { 0xAB, 0xDD, 0xA4 },{ 0x66, 0xC2, 0xA5 },
    { 0x32, 0x88, 0xBD },{ 0x5E, 0x4F, 0xA2 }*/
    { 0x27, 0x01, 0x10 },{ 0x35, 0x0F, 0x13 }, 
    { 0x3D, 0x1B, 0x10 },{ 0x3F, 0x2B, 0x18 },
    { 0x3E, 0x38, 0x22 },{ 0x39, 0x3d, 0x26 },
    { 0x2A, 0x37, 0x29 },{ 0x19, 0x30, 0x29 },
    { 0x0C, 0x22, 0x2F },{ 0x17, 0x13, 0x28 } 
};

//status bar colors
static unsigned char sb_colors[10][3] = {
   /* { 0x77, 0xE0, 0x5C },{ 0x00, 0xE8, 0xA1 }, 
    { 0x00, 0xEA, 0xEC },{ 0x00, 0xE4, 0xFF },
    { 0x00, 0xD5, 0xFF },{ 0xBD, 0xBB, 0xFF },
    { 0xFF, 0x9C, 0xFF },{ 0xFF, 0x87, 0xD9 },
    { 0xFF, 0x8E, 0x92 },{ 0xFE, 0xA9, 0x54 } */
    { 0x1D, 0x28, 0x17 },{ 0x00, 0x2D, 0x28 }, 
    { 0x00, 0x33, 0x3B },{ 0x05, 0x3A, 0x3F },
    { 0x10, 0x35, 0x3F },{ 0x1F, 0x2E, 0x3F },
    { 0x1F, 0x36, 0x3F },{ 0x2F, 0x21, 0x34 },
    { 0x2F, 0x23, 0x24 },{ 0x3F, 0x2A, 0x15 } 
};

//player center colors
static unsigned char center_colors[10][3] = {
    /*{ 0xFF, 0x89, 0x8C },{ 0xF6, 0xD9, 0x73 },  
    { 0x88, 0xDE, 0x71 },{ 0x00, 0xE6, 0x87 },   
    { 0x00, 0xEA, 0xC5 },{ 0x00, 0xE9, 0xFF },
    { 0x2A, 0xC8, 0xFF },{ 0x48, 0xAC, 0xFF },
    { 0xA8, 0xA0, 0xFF },{ 0xF3, 0xAA, 0xFF } */
    /////////////
    { 0x3F, 0x22, 0x23 },{ 0x3D, 0x36, 0x1C },  
    { 0x22, 0x37, 0x1C },{ 0x00, 0x39, 0x21 },   
    { 0x00, 0x3A, 0x31 },{ 0x00, 0x3A, 0x3F },
    { 0x0A, 0x32, 0x3F },{ 0x12, 0x2B, 0x3F },
    { 0x2A, 0x28, 0x3F },{ 0x2C, 0x1A, 0x2F } 
};

int fruit_timer;    //counts how long fruit text is on screen
int fruit_type;     //type of fruit

/* 
 * prepare_maze_level
 *   DESCRIPTION: Prepare for a maze of a given level.  Fills the game_info
 *          structure, creates a maze, and initializes the display.
 *   INPUTS: level -- level to be used for selecting parameter values
 *   OUTPUTS: none
 *   RETURN VALUE: 0 on success, -1 on failure
 *   SIDE EFFECTS: writes entire game_info structure; changes maze;
 *                 initializes display
 */
static int prepare_maze_level(int level) {
    int i; /* loop index for drawing display */
    
    /*
     * Record level in game_info; other calculations use offset from
     * level 1.
     */
    game_info.number = level--;

    /* Set per-level parameter values. */
    if ((game_info.maze_x_dim = MAZE_MIN_X_DIM + 2 * level) > MAZE_MAX_X_DIM)
        game_info.maze_x_dim = MAZE_MAX_X_DIM;
    if ((game_info.maze_y_dim = MAZE_MIN_Y_DIM + 2 * level) > MAZE_MAX_Y_DIM)
        game_info.maze_y_dim = MAZE_MAX_Y_DIM;
    if ((game_info.initial_fruit_count = 1 + level / 2) > 6)
        game_info.initial_fruit_count = 6;
    if ((game_info.time_to_first_fruit = 300 - 30 * level) < 120)
        game_info.time_to_first_fruit = 120;
    if ((game_info.time_between_fruits = 300 - 60 * level) < 60)
        game_info.time_between_fruits = 60;
    if ((game_info.tick_usec = 20000 - 1750 * level) < 5000)
        game_info.tick_usec = 5000;

    /* Initialize dynamic values. */
    game_info.map_x = game_info.map_y = SHOW_MIN;

    /* Create a maze. */
    if (make_maze(game_info.maze_x_dim, game_info.maze_y_dim, game_info.initial_fruit_count) != 0)
        return -1;
    
    /* Set logical view and draw initial screen. */
    set_view_window(game_info.map_x, game_info.map_y);
    for (i = 0; i < SCROLL_Y_DIM; i++)
        (void)draw_horiz_line (i);

    /* Return success. */
    return 0;
}

/* 
 * move_up
 *   DESCRIPTION: Move the player up one pixel (assumed to be a legal move)
 *   INPUTS: ypos -- pointer to player's y position (pixel) in the maze
 *   OUTPUTS: *ypos -- reduced by one from initial value
 *   RETURN VALUE: none
 *   SIDE EFFECTS: pans display by one pixel when appropriate
 */
static void move_up(int* ypos) {
    /*
     * Move player by one pixel and check whether display should be panned.
     * Panning is necessary when the player moves past the upper pan border
     * while the top pixels of the maze are not on-screen.
     */
    if (--(*ypos) < game_info.map_y + BLOCK_Y_DIM * PAN_BORDER && game_info.map_y > SHOW_MIN) {
        /*
         * Shift the logical view upwards by one pixel and draw the
         * new line.
         */
        set_view_window(game_info.map_x, --game_info.map_y);
        (void)draw_horiz_line(0);
    }
}

/* 
 * move_right
 *   DESCRIPTION: Move the player right one pixel (assumed to be a legal move)
 *   INPUTS: xpos -- pointer to player's x position (pixel) in the maze
 *   OUTPUTS: *xpos -- increased by one from initial value
 *   RETURN VALUE: none
 *   SIDE EFFECTS: pans display by one pixel when appropriate
 */
static void move_right(int* xpos) {
    /*
     * Move player by one pixel and check whether display should be panned.
     * Panning is necessary when the player moves past the right pan border
     * while the rightmost pixels of the maze are not on-screen.
     */
    if (++(*xpos) > game_info.map_x + SCROLL_X_DIM - BLOCK_X_DIM * (PAN_BORDER + 1) &&
        game_info.map_x + SCROLL_X_DIM < (2 * game_info.maze_x_dim + 1) * BLOCK_X_DIM - SHOW_MIN) {
        /*
         * Shift the logical view to the right by one pixel and draw the
         * new line.
         */
        set_view_window(++game_info.map_x, game_info.map_y);
        (void)draw_vert_line(SCROLL_X_DIM - 1);
    }
}

/* 
 * move_down
 *   DESCRIPTION: Move the player right one pixel (assumed to be a legal move)
 *   INPUTS: ypos -- pointer to player's y position (pixel) in the maze
 *   OUTPUTS: *ypos -- increased by one from initial value
 *   RETURN VALUE: none
 *   SIDE EFFECTS: pans display by one pixel when appropriate
 */
static void move_down(int* ypos) {
    /*
     * Move player by one pixel and check whether display should be panned.
     * Panning is necessary when the player moves past the right pan border
     * while the bottom pixels of the maze are not on-screen.
     */
    if (++(*ypos) > game_info.map_y + SCROLL_Y_DIM - BLOCK_Y_DIM * (PAN_BORDER + 1) && 
        game_info.map_y + SCROLL_Y_DIM < (2 * game_info.maze_y_dim + 1) * BLOCK_Y_DIM - SHOW_MIN) {
        /*
         * Shift the logical view downwards by one pixel and draw the
         * new line.
         */
        set_view_window(game_info.map_x, ++game_info.map_y);
        (void)draw_horiz_line(SCROLL_Y_DIM - 1);
    }
}

/* 
 * move_left
 *   DESCRIPTION: Move the player right one pixel (assumed to be a legal move)
 *   INPUTS: xpos -- pointer to player's x position (pixel) in the maze
 *   OUTPUTS: *xpos -- decreased by one from initial value
 *   RETURN VALUE: none
 *   SIDE EFFECTS: pans display by one pixel when appropriate
 */
static void move_left(int* xpos) {
    /*
     * Move player by one pixel and check whether display should be panned.
     * Panning is necessary when the player moves past the left pan border
     * while the leftmost pixels of the maze are not on-screen.
     */
    if (--(*xpos) < game_info.map_x + BLOCK_X_DIM * PAN_BORDER && game_info.map_x > SHOW_MIN) {
        /*
         * Shift the logical view to the left by one pixel and draw the
         * new line.
         */
        set_view_window(--game_info.map_x, game_info.map_y);
        (void)draw_vert_line (0);
    }
}

/* 
 * unveil_around_player
 *   DESCRIPTION: Show the maze squares in an area around the player.
 *                Consume any fruit under the player.  Check whether
 *                player has won the maze level.
 *   INPUTS: (play_x,play_y) -- player coordinates in pixels
 *   OUTPUTS: none
 *   RETURN VALUE: 1 if player wins the level by entering the square
 *                 0 if not
 *   SIDE EFFECTS: draws maze squares for newly visible maze blocks,
 *                 consumed fruit, and maze exit; consumes fruit and
 *                 updates displayed fruit counts
 */
static int unveil_around_player(int play_x, int play_y) {
    int x = play_x / BLOCK_X_DIM; /* player's maze lattice position */
    int y = play_y / BLOCK_Y_DIM;
    int i, j;            /* loop indices for unveiling maze squares */

    /* Check for fruit at the player's position. */
    fruit_type = check_for_fruit (x, y);

    //set fruit timer to value (128, its 2^)
    if(fruit_type != 0){
      fruit_timer = FRUIT_TIMER;
    }

    /* Unveil spaces around the player. */
    for (i = -1; i < 2; i++)
        for (j = -1; j < 2; j++)
            unveil_space(x + i, y + j);
        unveil_space(x, y - 2);
        unveil_space(x + 2, y);
        unveil_space(x, y + 2);
        unveil_space(x - 2, y);

    /* Check whether the player has won the maze level. */
    return check_for_win (x, y);
}

#ifndef NDEBUG
/* 
 * sanity_check 
 *   DESCRIPTION: Perform checks on changes to constants and enumerated values.
 *   INPUTS: none
 *   OUTPUTS: none
 *   RETURN VALUE: 0 if checks pass, -1 if any fail
 *   SIDE EFFECTS: none
 */
static int sanity_check() {
    /* 
     * Automatically detect when fruits have been added in blocks.h
     * without allocating enough bits to identify all types of fruit
     * uniquely (along with 0, which means no fruit).
     */
    if (((2 * LAST_MAZE_FRUIT_BIT) / MAZE_FRUIT_1) < NUM_FRUIT_TYPES + 1) {
        puts("You need to allocate more bits in maze_bit_t to encode fruit.");
        return -1;
    }
    return 0;
}
#endif /* !defined(NDEBUG) */

// Shared Global Variables
int quit_flag = 0;
int winner= 0;
int next_dir = UP;
int play_x, play_y, last_dir, dir;
int move_cnt = 0;
int fd;
int fe;

unsigned long data;
static struct termios tio_orig;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;

/*
 * keyboard_thread
 *   DESCRIPTION: Thread that handles keyboard inputs
 *   INPUTS: none
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
static void *keyboard_thread(void *arg) {
    char key;
    int state = 0;
    // Break only on win or quit input - '`'
    while (winner == 0) {        
        // Get Keyboard Input
        key = getc(stdin);
        
        // Check for '`' to quit
        if (key == BACKQUOTE) {
            quit_flag = 1;
            break;
        }
        
        // Compare and Set next_dir
        // Arrow keys deliver 27, 91, ##
        if (key == 27) {
            state = 1;
        }
        else if (key == 91 && state == 1) {
            state = 2;
        }
        else {    
            if (key >= UP && key <= LEFT && state == 2) {
                pthread_mutex_lock(&mtx);
                switch(key) {
                    case UP:
                        next_dir = DIR_UP;
                        break;
                    case DOWN:
                        next_dir = DIR_DOWN;
                        break;
                    case RIGHT:
                        next_dir = DIR_RIGHT;
                        break;
                    case LEFT:
                        next_dir = DIR_LEFT;
                        break;
                }
                pthread_mutex_unlock(&mtx);
            }
            state = 0;
        }
    }

    return 0;
}

/* some stats about how often we take longer than a single timer tick */
static int goodcount = 0;
static int badcount = 0;
static int total = 0;

/*
 * rtc_thread
 *   DESCRIPTION: Thread that handles updating the screen
 *   INPUTS: none
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
static void *rtc_thread(void *arg) {
    int ticks = 0;
    int level;
    int ret;
    int open[NUM_DIRS];
    int need_redraw = 0;
    int goto_next_level = 0;
    unsigned int min;
    unsigned int sec;
    unsigned int time;
    unsigned char sb_buf[SB_BUF_SIZE];        //status bar buffer
    unsigned char background_buf[BACKGROUND_BUF_SIZE][BACKGROUND_BUF_SIZE];   //masking background buffer 

    //text to print based on fruit type
    char *fruit_text_list[NUM_FRUIT_TYPES] = {"an apple!", "grapes!", "a peach!", "a strawberry!", "a banana!", "watermelon!", "YEAH! DEW!"};
    char *fruit = "";        //holds correct fruit text
    unsigned char fruit_txt_background[FRUIT_TXT_X_DIM*FRUIT_TXT_Y_DIM];    //fruit text background buffer
    unsigned char fruit_txt[FRUIT_TXT_X_DIM*FRUIT_TXT_Y_DIM];           //fruit text buffer
    int fruit_txt_x = 0; int fruit_txt_y = 0;
    
    int color_time = 0;     //index of center color
    int r; int g; int b;    //rgb vars for transparent colors

    // Loop over levels until a level is lost or quit.
    for (level = 1; (level <= MAX_LEVEL) && (quit_flag == 0); level++) {
        // Prepare for the level.  If we fail, just let the player win.
        if (prepare_maze_level(level) != 0)
            break;
        goto_next_level = 0;

        tux_timer(0, 0);        //set tux timer to 0 (just in case)
        // Start the player at (1,1)
        play_x = BLOCK_X_DIM;
        play_y = BLOCK_Y_DIM;

        // move_cnt tracks moves remaining between maze squares.
        // When not moving, it should be 0.
        move_cnt = 0;

        // Initialize last direction moved to up
        last_dir = DIR_UP;

        // Initialize the current direction of motion to stopped
        dir = DIR_STOP;
        next_dir = DIR_STOP;

        //initalize status bar
        min = 0;
        sec = 0;
        time = 0;
        status_bar(sb_buf, level, min, sec);

        //set wall colors
        //color for level 1 is at [0][1:3]
        set_palette(WALL_FILL_COLOR, wall_colors[level-1][0], wall_colors[level-1][1], wall_colors[level-1][2]);
        set_palette(STATUS_BAR_COLOR, sb_colors[level-1][0], sb_colors[level-1][1], sb_colors[level-1][2]);
        
        //get transparent colors
        r = (wall_colors[level-1][0] + WHITE)/2;
        g = (wall_colors[level-1][1] + WHITE)/2;
        b = (wall_colors[level-1][2] + WHITE)/2;
        //set transparent colors
        set_palette(WALL_FILL_COLOR+TRANSPARENT, r, g, b);

        // Show maze around the player's original position
        (void)unveil_around_player(play_x, play_y);
        fruit_txt_x = play_x;
        fruit_txt_y = play_y;
        
        //display maze + status bar
        background_buffer(play_x, play_y, background_buf);
        draw_player(play_x, play_y, get_player_block(last_dir), get_player_mask(last_dir));
        draw_text(sb_buf, SB_BUF_WIDTH);
        show_screen();
        draw_full_block(play_x, play_y, *background_buf);

    
        // get first Periodic Interrupt
        ret = read(fd, &data, sizeof(unsigned long));

        while ((quit_flag == 0) && (goto_next_level == 0)) {
            // Wait for Periodic Interrupt
            ret = read(fd, &data, sizeof(unsigned long));

            //change player center color
            if (time == 16){            //change player center color every half second
                if (color_time > 10){   //if at last color
                    color_time = 0;     //go back to 1st color
                }
                //set player center color
                set_palette(PLAYER_CENTER_COLOR, center_colors[color_time][0], center_colors[color_time][1], center_colors[color_time][2]);
                //get transparent colors
                r = (center_colors[color_time][0] + WHITE)/2;
                g = (center_colors[color_time][1] + WHITE)/2;
                b = (center_colors[color_time][2] + WHITE)/2;
                //set transparents colors
                set_palette(PLAYER_CENTER_COLOR+TRANSPARENT, r, g, b);

                color_time++;
            }

            //based on update_rate = 32
            //keep track of time
            if (time < 32){
                time++;
            }
            else{
                time = 0;           //reset second counter
                sec++;              //increase seconds
                need_redraw = 1;    //update screen
            }
            if (sec == 60){         //60 seconds in 1 minute
                sec = 0;            //reset seconds
                min++;              //increase minutes
                need_redraw = 1;
            }

            tux_timer(min, sec);
            // Update tick to keep track of time.  If we missed some
            // interrupts we want to update the player multiple times so
            // that player velocity is smooth
            ticks = data >> 8;    

            total += ticks;

            // If the system is completely overwhelmed we better slow down:
            if (ticks > 8) ticks = 8;

            if (ticks > 1) {
                badcount++;
            }
            else {
                goodcount++;
            }

            while (ticks--) {

                // Lock the mutex
                pthread_mutex_lock(&mtx);

                // Check to see if a key has been pressed
                if (next_dir != dir) {
                    // Check if new direction is backwards...if so, do immediately
                    if ((dir == DIR_UP && next_dir == DIR_DOWN) ||
                        (dir == DIR_DOWN && next_dir == DIR_UP) ||
                        (dir == DIR_LEFT && next_dir == DIR_RIGHT) ||
                        (dir == DIR_RIGHT && next_dir == DIR_LEFT)) {
                        if (move_cnt > 0) {
                            if (dir == DIR_UP || dir == DIR_DOWN)
                                move_cnt = BLOCK_Y_DIM - move_cnt;
                            else
                                move_cnt = BLOCK_X_DIM - move_cnt;
                        }
                        dir = next_dir;
                    }
                }
                // New Maze Square!
                if (move_cnt == 0) {
                        
                    // The player has reached a new maze square; unveil nearby maze
                    // squares and check whether the player has won the level.
                    if (unveil_around_player(play_x, play_y)) {
                        pthread_mutex_unlock(&mtx);
                        goto_next_level = 1;
                        fruit_timer = 0;
                        break;
                    }
                
                    // Record directions open to motion.
                    find_open_directions (play_x / BLOCK_X_DIM, play_y / BLOCK_Y_DIM, open);
        
                    // Change dir to next_dir if next_dir is open 
                    if (open[next_dir]) {
                        dir = next_dir;
                    }
    
                    // The direction may not be open to motion...
                    //   1) ran into a wall
                    //   2) initial direction and its opposite both face walls
                    if (dir != DIR_STOP) {
                        if (!open[dir]) {
                            dir = DIR_STOP;
                        }
                        else if (dir == DIR_UP || dir == DIR_DOWN) {    
                            move_cnt = BLOCK_Y_DIM;
                        }
                        else {
                            move_cnt = BLOCK_X_DIM;
                        }
                    }
                }
                // Unlock the mutex
                pthread_mutex_unlock(&mtx);
        
                if (dir != DIR_STOP) {
                    // move in chosen direction
                    last_dir = dir;
                    move_cnt--;    
                    switch (dir) {
                        case DIR_UP:    
                            move_up(&play_y);    
                            break;
                        case DIR_RIGHT: 
                            move_right(&play_x); 
                            break;
                        case DIR_DOWN:  
                            move_down(&play_y);  
                            break;
                        case DIR_LEFT:  
                            move_left(&play_x);  
                            break;
                    }
                    //draw_full_block(play_x, play_y, *background_buf);    
                    need_redraw = 1;
                }
            }

            //get fruit text coordinates
            fruit_txt_x = play_x - (FRUIT_TXT_X_DIM/2);
            fruit_txt_y = play_y - FRUIT_TXT_Y_DIM;

            //if a fruit has been picked up
            if (fruit_type != 0){
                fruit = fruit_text_list[fruit_type-1];
            }

            if (fruit_timer > 0){
                //get background, convert string to buffer, show floating text
                get_txt_back(fruit_txt_x, fruit_txt_y, fruit_txt, fruit_txt_background);
                string_to_font_fruit(fruit, fruit_txt, fruit_txt_background);
                draw_txt_fruit(fruit_txt_x, fruit_txt_y, fruit_txt);
            }            

            //redraw background
            background_buffer(play_x, play_y, background_buf);
            draw_player(play_x, play_y, get_player_block(last_dir), get_player_mask(last_dir));
            status_bar(sb_buf, level, min, sec);                    //update status bar
            draw_text(sb_buf, SB_BUF_WIDTH);                        //display status bar
            show_screen();    
            draw_full_block(play_x, play_y, *background_buf);
            
            //update floating text
            if (fruit_timer > 0){
                draw_txt_fruit(fruit_txt_x, fruit_txt_y, fruit_txt_background);
                fruit_timer--;
            }
            need_redraw = 0;
        }    
    }
    if (quit_flag == 0)
        winner = 1;
    
    return 0;
}


/*
 * status_bar
 *   DESCRIPTION: update string with new values of the variables for the status bar
 *   INPUTS: sb_buf[] -- buffer that will have the graphical image of string
 *           level -- current level, to be displayed in status bar
 *           minutes -- current minute count, to be displayed in status bar
 *           seconds -- current second count, to be displayed in status bar
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: updates the image in sb_buf, to be drawn to the status bar
 */
static void status_bar(unsigned char sb_buf[], unsigned int level, unsigned int minutes, unsigned int seconds){      //get/set status bar values

    //init variables    
    char str[40];
    const char *string;      
    char str_lvl[3];
    char str_fruit[3];
    char sec[3];
    char min[3];

    int fruit = get_num_fruits();      //get number of fruits

    //convert variables to strings
    sprintf(str_fruit, "%d", fruit);   
    sprintf(str_lvl, "%d", level);
    sprintf(sec, "%02d", seconds);
    sprintf(min, "%02d", minutes);

    //concat string together
    strcpy(str, "Level ");
    strcat(str, str_lvl);
    strcat(str, "  ");
    strcat(str, str_fruit);
    strcat(str, " Fruit");
    strcat(str, "  ");
    strcat(str, min);
    strcat(str, ":");
    strcat(str, sec);
    string = str;

    //make string into buffer
    string_to_font(string, sb_buf, WHITE, STATUS_BAR_COLOR);
};

/*
 * tux_thread
 *   DESCRIPTION: thread that handels tux inputs
 *   INPUTS: none
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
static void *tux_thread(void *arg){

    unsigned long buttons;
    // Break only on win or quit input - '`'
    while (winner == 0) {        
        // Get button Input
        
        ioctl(fe, TUX_BUTTONS, &buttons);
        //start = quit
        if (buttons == T_START) {
            quit_flag = 1;
            break;
        }
        // Compare and Set next_dir
        if (quit_flag == 1 || winner == 1) {
            break;
        }
                //button inputs
                pthread_mutex_lock(&mtx);
                switch(buttons) {
                    case T_UP:
                        next_dir = DIR_UP;
                        break;
                    case T_DOWN:
                        next_dir = DIR_DOWN;
                        break;
                    case T_RIGHT:
                        next_dir = DIR_RIGHT;
                        break;
                    case T_LEFT:
                        next_dir = DIR_LEFT;
                        break;
                    case T_START:
                        break;
                    default:
                        break;
                }
                pthread_mutex_unlock(&mtx);

    }

    return 0;

};

/*
 * tux_timer
 *   DESCRIPTION: divide up the time to send to tux to display on LEDs
 *   INPUTS: min -- current minute count, displayed on LED3 & LED2
 *           sec -- current second count, displayed on LED1 & LED0
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: displays time on tux LEDs
 */
static void tux_timer(int min, int sec)
{
	int time;

    //start with 3 LEDs showing 0 (like in demo)
	time = 0x04070000;      

    //seperate time into the 4 sections
    time = time | (sec % 10);                           //LED0, sec2 aka hundredths place
	time = time | (((sec - (sec % 10)) / 10) << 4);     //LED1, sec1 aka tenths place
	time = time | ((min % 10) << 8);                    //LED2, min2 aka ones place
	time = time | (((min - (min % 10)) / 10) << 12);    //LED3, min1 aka tens place
	
    //display on TUX
	ioctl(fe, TUX_SET_LED, time);
}




/*
 * main
 *   DESCRIPTION: Initializes and runs the two threads
 *   INPUTS: none
 *   OUTPUTS: none
 *   RETURN VALUE: 0 on success, -1 on failure
 *   SIDE EFFECTS: none
 */
int main() {
    int ret;
    struct termios tio_new;
    unsigned long update_rate = 32; /* in Hz */

    pthread_t tid1;
    pthread_t tid2;
    pthread_t tid3;

    // Initialize RTC
    fd = open("/dev/rtc", O_RDONLY, 0);
    
    // Enable RTC periodic interrupts at update_rate Hz
    // Default max is 64...must change in /proc/sys/dev/rtc/max-user-freq
    ret = ioctl(fd, RTC_IRQP_SET, update_rate);    
    ret = ioctl(fd, RTC_PIE_ON, 0);

    //initilize tux
    fe = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
    int ldisc_num = N_MOUSE;
    ioctl(fe, TIOCSETD, &ldisc_num);
    ioctl(fe, TUX_INIT);

    // Initialize Keyboard
    // Turn on non-blocking mode
    if (fcntl(fileno(stdin), F_SETFL, O_NONBLOCK) != 0) {
        perror("fcntl to make stdin non-blocking");
        return -1;
    }
    
    // Save current terminal attributes for stdin.
    if (tcgetattr(fileno(stdin), &tio_orig) != 0) {
        perror("tcgetattr to read stdin terminal settings");
        return -1;
    }
    
    // Turn off canonical (line-buffered) mode and echoing of keystrokes
    // Set minimal character and timing parameters so as
    tio_new = tio_orig;
    tio_new.c_lflag &= ~(ICANON | ECHO);
    tio_new.c_cc[VMIN] = 1;
    tio_new.c_cc[VTIME] = 0;
    if (tcsetattr(fileno(stdin), TCSANOW, &tio_new) != 0) {
        perror("tcsetattr to set stdin terminal settings");
        return -1;
    }

    // Perform Sanity Checks and then initialize input and display
    if ((sanity_check() != 0) || (set_mode_X(fill_horiz_buffer, fill_vert_buffer) != 0)){
        return 3;
    }

    // Create the threads
    pthread_create(&tid1, NULL, rtc_thread, NULL);
    pthread_create(&tid2, NULL, keyboard_thread, NULL);
    pthread_create(&tid3, NULL, tux_thread, NULL);
    
    // Wait for all the threads to end
    pthread_join(tid1, NULL);
    pthread_join(tid2, NULL);
    pthread_join(tid3, NULL);

    // Shutdown Display
    clear_mode_X();
    
    // Close Keyboard
    (void)tcsetattr(fileno(stdin), TCSANOW, &tio_orig);
        
    // Close RTC
    close(fd);

    //close TUX
    close(fe);

    // Print outcome of the game
    if (winner == 1) {    
        printf("You win the game! CONGRATULATIONS!\n");
    }
    else if (quit_flag == 1) {
        printf("Quitter!\n");
    }
    else {
        printf ("Sorry, you lose...\n");
    }

    // Return success
    return 0;
}
