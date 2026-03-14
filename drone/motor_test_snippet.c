/*
 * ESP32 Drone — Motor Test Snippet
 * =================================
 * 
 * Add this code to drone_main.c to test each motor individually.
 * 
 * USAGE:
 * 1. Copy this entire function into drone_main.c (before app_main)
 * 2. In app_main(), after motors_init(), uncomment: test_motor_sequence();
 * 3. Rebuild and flash
 * 4. Remove propellers and watch each motor spin for 2 seconds
 * 5. Listen for stuttering, jamming, or uneven speed
 * 
 * MOTOR LAYOUT (top view):
 *         FRONT
 *     M3(FL,CCW)  M1(FR,CW)
 *          \\      //
 *           \\    //
 *            ====
 *           //    \\
 *          //      \\
 *     M4(BL,CW)   M2(BR,CCW)
 *          BACK
 */

static void test_motor_sequence(void) {
  ESP_LOGI(TAG, "╔════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║      MOTOR DIAGNOSTIC TEST SEQUENCE        ║");
  ESP_LOGI(TAG, "║    Each motor spins at 1200µs for 2 sec    ║");
  ESP_LOGI(TAG, "║    REMOVE PROPELLERS BEFORE THIS TEST!     ║");
  ESP_LOGI(TAG, "╚════════════════════════════════════════════╝");
  
  const struct {
    int idx;
    const char *name;
    const char *pos;
    const char *dir;
    int gpio;
  } motors[4] = {
      {0, "Motor 1", "Front-Right", "CW", MOTOR1_GPIO},
      {1, "Motor 2", "Back-Right", "CCW", MOTOR2_GPIO},
      {2, "Motor 3", "Front-Left", "CCW", MOTOR3_GPIO},
      {3, "Motor 4", "Back-Left", "CW", MOTOR4_GPIO},
  };
  
  /* Zero all motors first */
  uint16_t zero[4] = {MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};
  motors_set(zero);
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  for (int i = 0; i < 4; i++) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "┌────────────────────────────────────────────┐");
    ESP_LOGI(TAG, "│ Testing: %s (%s, %s) GPIO %d", 
             motors[i].name, motors[i].pos, motors[i].dir, motors[i].gpio);
    ESP_LOGI(TAG, "├────────────────────────────────────────────┤");
    
    /* Zero all motors */
    motors_set(zero);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    /* Spin only this motor at 1200µs (low throttle) */
    uint16_t test[4] = {MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};
    test[i] = 1200;
    motors_set(test);
    
    ESP_LOGI(TAG, "│ Spinning at 1200µs (25% power)...");
    
    /* Run for 2 seconds, log every 200ms */
    for (int tick = 0; tick < 10; tick++) {
      vTaskDelay(pdMS_TO_TICKS(200));
      int pct = ((tick + 1) * 100) / 10;
      ESP_LOGI(TAG, "│ Running... [%-50s] %d%%", 
               "========================================", pct);
    }
    
    /* Stop and report */
    motors_set(zero);
    ESP_LOGI(TAG, "│ Test complete. Listen for:");
    ESP_LOGI(TAG, "│   ✓ Smooth, continuous rotation");
    ESP_LOGI(TAG, "│   ✓ No stuttering or jerking");
    ESP_LOGI(TAG, "│   ✗ No grinding, clicking, or jamming");
    ESP_LOGI(TAG, "└────────────────────────────────────────────┘");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "╔════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║    MOTOR TEST COMPLETE                     ║");
  ESP_LOGI(TAG, "║                                            ║");
  ESP_LOGI(TAG, "║  Did all motors spin smoothly?             ║");
  ESP_LOGI(TAG, "║    YES → Continue flying (go to Step 2)    ║");
  ESP_LOGI(TAG, "║    NO  → Replace the faulty motor/ESC      ║");
  ESP_LOGI(TAG, "║                                            ║");
  ESP_LOGI(TAG, "║  Comment out test_motor_sequence() call    ║");
  ESP_LOGI(TAG, "║  Rebuild and flash to resume normal ops    ║");
  ESP_LOGI(TAG, "╚════════════════════════════════════════════╝");
  
  /* Keep this task alive but do nothing */
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/*
 * STEP-BY-STEP INSTRUCTIONS:
 * 
 * 1. OPEN esp/drone/main/drone_main.c
 * 
 * 2. ADD THIS FUNCTION (copy the entire test_motor_sequence above)
 *    Place it BEFORE the app_main() function
 * 
 * 3. MODIFY app_main() to call the test:
 *    Find the line: motors_init();
 *    Right after it, add:
 *      test_motor_sequence();  // <-- ADD THIS LINE
 * 
 * 4. BUILD AND FLASH:
 *    cd esp/drone
 *    idf.py build
 *    idf.py -p /dev/ttyUSB0 flash
 *    idf.py -p /dev/ttyUSB0 monitor
 * 
 * 5. WATCH THE SERIAL OUTPUT
 *    You should see:
 *    - Motor 1 spins for 2 seconds
 *    - Motor 2 spins for 2 seconds
 *    - Motor 3 spins for 2 seconds
 *    - Motor 4 spins for 2 seconds
 *    - Then it says TEST COMPLETE
 * 
 * 6. LISTEN CAREFULLY
 *    All motors should sound similar:
 *    - Smooth continuous whine
 *    - No stuttering or jerking
 *    - No grinding or clicking
 * 
 *    If one motor sounds different:
 *    - It might be faulty
 *    - Try reconnecting the ESC/motor wires
 *    - If still bad, replace that ESC or motor
 * 
 * 7. AFTER TESTING:
 *    - Comment out the test_motor_sequence() call
 *    - Delete this test function
 *    - Rebuild and flash normal firmware
 * 
 * MOTOR POSITIONS (X-quadcopter, top view):
 * 
 *           FRONT
 *       M3 ←─┬─→ M1
 *       CCW  │   CW
 *           BODY
 *       CW   │   CCW
 *       M4 ←─┴─→ M2
 *           BACK
 * 
 * Expected rotation directions:
 *   M1 (Front-Right):  Clockwise (↗)
 *   M2 (Back-Right):   Counter-Clockwise (↙)
 *   M3 (Front-Left):   Counter-Clockwise (↖)
 *   M4 (Back-Left):    Clockwise (↘)
 * 
 * If a motor spins the wrong way, swap any two of its three wires.
 */