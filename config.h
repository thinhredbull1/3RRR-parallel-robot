#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2600
#define DEFAULT_PULSE_WIDTH 1500
#define REFRESE_INTERVAL 20000
#define servo1 0
#define servo2 1
#define servo3 2
#define PRINT_SERIAL_COMMA(...) SerialPrintComma(__VA_ARGS__)
#define rad_to_deg(x) x * 180.0 / M_PI
#define deg_to_rad(x) (M_PI/180.0)*x
template<typename T>
void SerialPrintComma(T t) {
    Serial.println(t);
}
#define run_every(t) for (static uint16_t last_; \
                          (uint16_t)(uint16_t(millis()) - last_) >= (t); \
                          last_ += (t))
template<typename T, typename... Args>
void SerialPrintComma(T t, Args... args) {
    Serial.print(t);  
    Serial.print(","); 
    SerialPrintComma(args...);
}
double theta_min[3] ={deg_to_rad(0),deg_to_rad(0),deg_to_rad(0)};
double theta_max[3] ={deg_to_rad(175),deg_to_rad(175),deg_to_rad(175)};

double m_of_theta_min[3]={650,650,650}; //
double m_of_theta_max[3]={2485,2485,2485}; //
const byte NumServos = 3;
float vmax[NumServos] = { 4, 4, 4};          // max change in p per step
float amax[NumServos] = {1.5, 1.5, 1.5};   // max change in v per step
byte group[NumServos] = { 1, 1, 1};
const double rp[3] = {0.045,0.045,0.045};
const double rb = 0.185;
const double L1 = 0.142;
const double L2 = 0.12;
const float InitialPosition[NumServos] = {
  37.5,
  27,
  38
};
const float origin_servo_angle[3]={deg_to_rad(99.63),deg_to_rad(99.63),deg_to_rad(99.63)};

struct coordinate{
    float x;
    float y;
    float theta;
};

float target[NumServos];  // target position
float p[NumServos];       // current position
float v[NumServos];       // velocity: (change in p per step)
const int VelPeriod = 20;
float sign(float a) {
  if (a < 0) return -1;
  if (a > 0) return +1;
  return 0;
}
float sqr(float a) {
  return a * a;
}
double interpolate(double x, double x_min, double x_max, double y_of_x_min, double y_of_x_max) {
  double y;
  y = (double) ((y_of_x_max-y_of_x_min)/(x_max-x_min)*(x-x_min) + y_of_x_min);
  return y;
}
float cvt_to_micro(float angle,int index) //degree{
{
  float micro_ =interpolate(deg_to_rad(angle), theta_min[index], theta_max[index], m_of_theta_min[index], m_of_theta_max[index]);
  return micro_;
}
