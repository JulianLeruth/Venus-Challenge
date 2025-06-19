#define BLACK                               0
#define WHITE                               1
#define RED                                 2
#define BLUE                                3
#define GREEN                               4

extern int tcs3472Ping(tcs3472* sensor, int integration_time_ms);

extern void printColour(uint16_t red, uint16_t green, uint16_t blue);

extern int colourSensor(tcs3472* sensor, int integration_time_ms);
