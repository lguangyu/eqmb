#include "hid_dev.h"

// special strings for the RAGE-BUTTON!

// ss1: YOU DID NOT RESPECT ME
const static keyboard_cmd_t ss1_w1[] = {HID_KEY_Y, HID_KEY_O, HID_KEY_U, 0x0};
const static keyboard_cmd_t ss1_w2[] = {HID_KEY_SPACEBAR, HID_KEY_D, HID_KEY_I, HID_KEY_D, 0x0};
const static keyboard_cmd_t ss1_w3[] = {HID_KEY_SPACEBAR, HID_KEY_N, HID_KEY_O, HID_KEY_T, 0x0};
const static keyboard_cmd_t ss1_w4[] = {HID_KEY_SPACEBAR, HID_KEY_R, HID_KEY_E, HID_KEY_S, 0x0};
const static keyboard_cmd_t ss1_w5[] = {HID_KEY_P, HID_KEY_E, HID_KEY_C, HID_KEY_T, 0x0};
const static keyboard_cmd_t ss1_w6[] = {HID_KEY_SPACEBAR, HID_KEY_M, HID_KEY_E, HID_KEY_DOT, 0x0};
const static keyboard_cmd_t *ss1[] = {ss1_w1, ss1_w2, ss1_w3, ss1_w4, ss1_w5, ss1_w6, NULL};

// ss2: I DON'T UNDERSTAND
const static keyboard_cmd_t ss2_w1[] = {HID_KEY_I, 0x0};
const static keyboard_cmd_t ss2_w2[] = {HID_KEY_SPACEBAR, HID_KEY_D, HID_KEY_O, HID_KEY_N, HID_KEY_SGL_QUOTE, HID_KEY_T, 0x0};
const static keyboard_cmd_t ss2_w3[] = {HID_KEY_SPACEBAR, HID_KEY_U, HID_KEY_N, HID_KEY_D, HID_KEY_E, HID_KEY_R, 0x0};
const static keyboard_cmd_t ss2_w4[] = {HID_KEY_S, HID_KEY_T, HID_KEY_A, HID_KEY_N, HID_KEY_D, HID_KEY_DOT, 0x0};
const static keyboard_cmd_t *ss2[] = {ss2_w1, ss2_w2, ss2_w3, ss2_w4, NULL};

// ss3: FROM READER'S PERSPECTIVE
const static keyboard_cmd_t ss3_w1[] = {HID_KEY_F, HID_KEY_R, HID_KEY_O, HID_KEY_M, 0x0};
const static keyboard_cmd_t ss3_w2[] = {HID_KEY_SPACEBAR, HID_KEY_R, HID_KEY_E, HID_KEY_A, HID_KEY_D, 0x0};
const static keyboard_cmd_t ss3_w3[] = {HID_KEY_E, HID_KEY_R, HID_KEY_SGL_QUOTE, HID_KEY_S, 0x0};
const static keyboard_cmd_t ss3_w4[] = {HID_KEY_SPACEBAR, HID_KEY_P, HID_KEY_E, HID_KEY_R, 0x0};
const static keyboard_cmd_t ss3_w5[] = {HID_KEY_S, HID_KEY_P, HID_KEY_E, HID_KEY_C, 0x0};
const static keyboard_cmd_t ss3_w6[] = {HID_KEY_T, HID_KEY_I, HID_KEY_V, HID_KEY_E, HID_KEY_DOT, 0x0};
const static keyboard_cmd_t *ss3[] = {ss3_w1, ss3_w2, ss3_w3, ss3_w4, ss3_w5, ss3_w6, NULL};

// ss4: I'M READING YOUR PAPER
const static keyboard_cmd_t ss4_w1[] = {HID_KEY_I, HID_KEY_SGL_QUOTE, HID_KEY_M, 0x0};
const static keyboard_cmd_t ss4_w2[] = {HID_KEY_SPACEBAR, HID_KEY_R, HID_KEY_E, HID_KEY_A, HID_KEY_D, 0x0};
const static keyboard_cmd_t ss4_w3[] = {HID_KEY_I, HID_KEY_N, HID_KEY_G, 0x0};
const static keyboard_cmd_t ss4_w4[] = {HID_KEY_SPACEBAR, HID_KEY_Y, HID_KEY_O, HID_KEY_U, HID_KEY_R, HID_KEY_SPACEBAR, 0x0};
const static keyboard_cmd_t ss4_w5[] = {HID_KEY_P, HID_KEY_A, HID_KEY_P, HID_KEY_E, HID_KEY_R, HID_KEY_DOT, 0x0};
const static keyboard_cmd_t *ss4[] = {ss4_w1, ss4_w2, ss4_w3, ss4_w4, ss4_w5, NULL};

const keyboard_cmd_t **eqmb_ragebtn_spec_str[] = {ss1, ss2, ss3, ss4};