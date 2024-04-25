#include <Arduino_FreeRTOS.h>
#include <math.h>

#define NB_SPEED 10 // Maximum number of speed limit changes
#define NB_DIRECTION 10 // Maximum number of crossings

struct SpeedLimit {
    double X;
    double Y;
    int Speed;
};

struct Crossing {
    double X;
    double Y;
    int Direction;
};

SpeedLimit SPEED[NB_SPEED] = {
    {1.234, 2.345, 60},
    {3.456, 4.567, 80},
    // Add other speed limits as necessary
};

Crossing DIRECTION[NB_DIRECTION] = {
    {5.678, 6.789, 0},
    {7.890, 8.901, 1},
    // Add other crossings as necessary
};

int pin1Distance = 1;
int pin2Direction = 2;
int pin3DispSpeed = 3;
int pin4ContBlinkSpeed = 4;
int pin5RedLight = 5;
int pinWarn6 = 6;

void setup() {
    Serial.begin(9600);

    pinMode(pin1Distance, OUTPUT);
    pinMode(pin2Direction, OUTPUT);
    pinMode(pin3DispSpeed, OUTPUT);
    pinMode(pin4ContBlinkSpeed, OUTPUT);
    pinMode(pin5RedLight, OUTPUT);
    pinMode(pinWarn6, INPUT);

    xTaskCreate(taskSpeedLimit, "SpeedLimit", 256, NULL, 1, NULL);
    xTaskCreate(taskCrossing, "Crossing", 256, NULL, 1, NULL);
    xTaskCreate(taskWarning, "Warning", 128, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() {
}

void taskSpeedLimit(void *pvParameters) {
    while (1) {
        int currentSpeed = readBikeSpeed();
        int speedLimit = getCurrentSpeedLimit();
        displaySpeed(speedLimit, currentSpeed != speedLimit);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskCrossing(void *pvParameters) {
    while (1) {
        int crossingIndex = getNextCrossingIndex();
        if (crossingIndex >= 0) {
            displayDirection(DIRECTION[crossingIndex].Direction);
        } else {
            displayDirection(-1); // No direction to display
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task to monitor and respond to warning signals
void taskWarning(void *pvParameters) {
    while (1) {
        if (digitalRead(pinWarn6) == HIGH) {
            digitalWrite(pin5RedLight, HIGH); // Turn on warning light if signal is high
        } else {
            digitalWrite(pin5RedLight, LOW); // Turn off warning light if signal is low
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 50 milliseconds before next check
    }
}

// Reads the current speed from a speed sensor connected to pin A0
int readBikeSpeed() {
    return analogRead(A0); // Placeholder for actual speed reading
}

// Determines the current speed limit based on the vehicle's GPS location
int getCurrentSpeedLimit() {
    double currentX = readGPSX();
    double currentY = readGPSY();
    for (int i = 0; i < NB_SPEED; i++) {
        if (isWithinSpeedLimitSection(SPEED[i].X, SPEED[i].Y, currentX, currentY)) {
            return SPEED[i].Speed; // Return speed limit if within a designated section
        }
    }
    return -1; // Return -1 if no speed limit is applicable
}

// Finds the index of the nearest speed limit zone based on the vehicle's current GPS location
int getNextSpeedLimitIndex() {
    double currentX = readGPSX();
    double currentY = readGPSY();
    int closestIndex = -1;
    double minDistance = DBL_MAX;
    for (int i = 0; i < NB_SPEED; i++) {
        double distance = calculateDistance(currentX, currentY, SPEED[i].X, SPEED[i].Y);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i; // Update closest index
        }
    }
    return closestIndex;
}

// Calculates the Euclidean distance between two GPS coordinates
double calculateDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)); // Return the calculated distance
}

// Checks if the vehicle is within a specified radius of a speed limit zone
bool isWithinSpeedLimitSection(double nextSpeedX, double nextSpeedY) {
    double currentX = readGPSX();
    double currentY = readGPSY();
    return calculateDistance(currentX, currentY, nextSpeedX, nextSpeedY) <= 50.0; // Check within 50 meters
}

// Determines the index of the nearest crossing based on the vehicle's current GPS location
int getNextCrossingIndex() {
    double currentX = readGPSX();
    double currentY = readGPSY();
    int closestIndex = -1;
    double minDistance = DBL_MAX;
    for (int i = 0; i < NB_DIRECTION; i++) {
        double distance = calculateDistance(currentX, currentY, DIRECTION[i].X, DIRECTION[i].Y);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i; // Update closest index
        }
    }
    return closestIndex;
}

// Displays the current speed and optionally blinks the display to alert the rider
void displaySpeed(int speed, bool blink) {
    Serial.print("Speed: ");
    Serial.println(speed);
    digitalWrite(pin3DispSpeed, HIGH);
    if (blink) {
        digitalWrite(pin4ContBlinkSpeed, HIGH); // Start blinking
        delay(250);
        digitalWrite(pin4ContBlinkSpeed, LOW); // Stop blinking
        delay(250);
    }
}

// Displays the direction information on the helmet's display
void displayDirection(int direction) {
    Serial.print("Direction: ");
    Serial.println(direction);
    digitalWrite(pin2Direction, HIGH); // Show direction on display
}



double readGPSX() {
    return 0.0; // Placeholder function to simulate reading GPS X-coordinate
}


double readGPSY() {
    return 0.0; // Placeholder function to simulate reading GPS Y-coordinate
}


double calculateDistanceToSpeedChange(int currentSpeed, int nextSpeed) {
    if (currentSpeed <= nextSpeed) {
        return 0.0; // No deceleration needed if speeding up or at the same speed
    }
    const double deceleration = -9.81; // Assuming negative for deceleration, m/s^2
    return (currentSpeed * currentSpeed - nextSpeed * nextSpeed) / (2 * deceleration);
}



int calculateTimeToSpeedChange(double distanceToSpeedChange) {
    const double deceleration = 9.81; // m/s^2, assuming constant deceleration
    double currentSpeed = readBikeSpeed();
    double finalSpeed = sqrt(currentSpeed * currentSpeed - 2 * deceleration * distanceToSpeedChange);
    return int((currentSpeed - finalSpeed) / deceleration);
}



int calculateTimeToCrossing(double nextCrossingX, double nextCrossingY) {
    double currentX = readGPSX();
    double currentY = readGPSY();
    double distance = calculateDistance(currentX, currentY, nextCrossingX, nextCrossingY);
    double currentSpeed = readBikeSpeed();
    if (currentSpeed == 0) return INT_MAX; // Prevent division by zero if stationary
    return int(distance / currentSpeed);
}
