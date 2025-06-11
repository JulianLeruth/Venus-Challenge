void waitTillDone(void);
int checkIfDone(void);

void straigth(int dist, uint16_t speed);
void turnRight(int angle, uint16_t speed);
void turnLeft(int angle, uint16_t speed);
void twist(int angle, uint16_t speed);
void straveRight(int angle, int dist, uint16_t speed);
void straveLeft(int angle, int dist, uint16_t speed);

void dance(void);

int sizeDetection(vl53x* sensor);
int objectDetectionTwist360(vl53x* sensor);
int objectDetectionTwistPart(vl53x* sensor, int angle);
int objectAproach(vl53x* sensor, int max_dist);
