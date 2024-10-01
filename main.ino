#include <Servo.h>
#include "config.h"
Servo sv1;
Servo sv2;
Servo sv3;
Servo* myservo[] = {
  &sv1,
  &sv2,
  &sv3
};

double px = 0.0;
double py = 0.0;
double theta = 0.0;
int out_of_reach;
bool ff;
float microsec;
int angle;
float home_offset_angle[3];
void MoveServos() {
  static unsigned long previousMillis = 0;
  if (millis() - previousMillis < VelPeriod)
    return;
  previousMillis = millis();

  float g[NumServos];
  float v1, a1, tmax, sign1, p0;

  for (byte grp = 0; grp < NumServos; grp++) {
    tmax = 0;
    for (byte servo = 0; servo < NumServos; servo++)
      if (group[servo] == grp) {
        g[servo] = abs(target[servo] - p[servo]) / vmax[servo];
        if (amax[servo] > 0) {
          sign1 = sign(sqr(v[servo]) + amax[servo] * 2 * (2 * abs(v[servo]) + sign(v[servo]) * (p[servo] - target[servo])));
          g[servo] = max(g[servo],
                         (float)(sign1 * abs(v[servo]) + sqrt(abs(2 * sign1 * v[servo] * abs(v[servo]) + 4 * amax[servo] * (p[servo] - target[servo])))) / amax[servo]);
        }
        tmax = max(tmax, g[servo]);
      }
    for (byte servo = 0; servo < NumServos; servo++)
      if (group[servo] == grp) {
        if (grp == 0)
          tmax = g[servo];
        if ((tmax < 2) or ((abs(v[servo]) <= amax[servo] + 0.0001) and (abs(p[servo] + v[servo] - target[servo]) <= 1))) {
          p[servo] = target[servo];
          v[servo] = 0;
        } else if (tmax > 0) {
          g[servo] = g[servo] / tmax;
          a1 = amax[servo] * sqr(g[servo]);

          if (a1 == 0) {
            v[servo] = (target[servo] - p[servo]) / tmax;
          } else {
            v1 = vmax[servo] * g[servo];
            p0 = 2 * v[servo] + abs(v[servo]) * v[servo] / (2 * a1);
            a1 = sign(target[servo] - p[servo] - p0) * a1;
            if (a1 * (target[servo] - p[servo]) < 0) {
              a1 = sign(a1) * abs(v[servo]) * v[servo] / (2 * (target[servo] - p[servo]));
              a1 = constrain(a1, -amax[servo], amax[servo]);
            }

            v[servo] = v[servo] + a1;
            v[servo] = constrain(v[servo], -v1, v1);
          }

          v[servo] = constrain(v[servo], -vmax[servo], vmax[servo]);
          p[servo] = p[servo] + v[servo];
        }
      }
  }
  for (byte servo = 0; servo < NumServos; servo++) {
    p[servo] = constrain(p[servo], rad_to_deg(theta_min[servo]), rad_to_deg(theta_max[servo]));
    float microsec_now = interpolate(deg_to_rad(p[servo]), theta_min[servo], theta_max[servo], m_of_theta_min[servo], m_of_theta_max[servo]);  //interpolates
    // if (servo == 1) PRINT_SERIAL_COMMA(p[servo], microsec_now);
    myservo[servo]->writeMicroseconds((int)microsec_now);
    // pwm.setPWM(servo, 0, cvt_angle(p[servo]));
    //Serial.println("servo[" + String(servo) + "]:" + String(p[servo]));
  }
}
void InvKinRRR(double px, double py, double theta) {
  double b1x = rb * cos(-M_PI / 6);
  double b1y = rb * sin(-M_PI / 6);
  double b2x = 0;
  double b2y = rb;
  double b3x = rb * cos(7 * M_PI / 6);
  double b3y = rb * sin(7 * M_PI / 6);
  double a_angles[3] = { -M_PI / 6, M_PI / 2, 7 * M_PI / 6 };
  double a1x = px + rp[0] * cos(theta + a_angles[0]);
  double a1y = py + rp[0] * sin(theta + a_angles[0]);
  double a2x = px + rp[1] * cos(theta + a_angles[1]);
  double a2y = py + rp[1] * sin(theta + a_angles[1]);
  double a3x = px + rp[2] * cos(theta + a_angles[2]);
  double a3y = py + rp[2] * sin(theta + a_angles[2]);

  double a1b1 = sqrt((a1x - b1x) * (a1x - b1x) + (a1y - b1y) * (a1y - b1y));
  double a2b2 = sqrt((a2x - b2x) * (a2x - b2x) + (a2y - b2y) * (a2y - b2y));
  double a3b3 = sqrt((a3x - b3x) * (a3x - b3x) + (a3y - b3y) * (a3y - b3y));

  double alpha1 = acos((a1b1 * a1b1 + L1 * L1 - L2 * L2) / (2 * L1 * a1b1));
  double alpha2 = acos((a2b2 * a2b2 + L1 * L1 - L2 * L2) / (2 * L1 * a2b2));
  double alpha3 = acos((a3b3 * a3b3 + L1 * L1 - L2 * L2) / (2 * L1 * a3b3));

  double psi1 = atan2(a1y - b1y, a1x - b1x);
  double psi2 = atan2(a2y - b2y, a2x - b2x);
  double psi3 = atan2(a3y - b3y, a3x - b3x);

  psi1 = unwrap(psi1);
  psi2 = unwrap(psi2);
  psi3 = unwrap(psi3);
  double q[3];
  q[0] = unwrap(psi1 - alpha1);
  q[1] = unwrap(psi2 - alpha2 - (double)(2 * M_PI / 3));
  q[2] = unwrap(psi3 - alpha3 - (double)(4 * M_PI / 3));
  out_of_reach = 0;  //initialize with 0
  if (isnan(q[0]) || isnan(q[1]) || isnan(q[2])) {
    out_of_reach = 1;
  }
  if (!out_of_reach)
    for (int i = 0; i < 3; i++) {
      target[i] = q[i] - home_offset_angle[i];
      target[i] = rad_to_deg(target[i]);
    }
  else Serial.println("OUT OF REACH");
}
//=============================================================
double unwrap(double theta) {

  if (theta < 0) {
    theta = theta + 2 * M_PI;
  }
  return theta;
}
coordinate Serial_process() {
  coordinate result;
  if (Serial.available()) {
    String c = Serial.readStringUntil(';');
    int index_now = c.indexOf("/");
    if (index_now != -1) {
      int index_cal = c.indexOf("#");
      result.x = c.substring(0, index_now).toFloat();
      result.y = c.substring(index_now + 1, index_cal).toFloat();
      result.theta = c.substring(index_cal + 1).toFloat();
      Serial.print("coord:");
      PRINT_SERIAL_COMMA(result.x, result.y, result.theta);
      Serial.print("angle:");
      InvKinRRR(result.x, result.y, deg_to_rad(result.theta));
      PRINT_SERIAL_COMMA(target[0], target[1], target[2]);
    }
  }
  return result;
}
void setup() {
  Serial.begin(9600);
  sv1.attach(D4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  sv2.attach(D5, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  sv3.attach(D6, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  for (int i = 0; i < 3; i++) {
    float microsec_now = cvt_to_micro(InitialPosition[i], i);  //interpolates  '
    myservo[i]->writeMicroseconds(microsec_now);
    p[i] = InitialPosition[i];
    target[i] = InitialPosition[i];
    home_offset_angle[i] = origin_servo_angle[i] - deg_to_rad(InitialPosition[i]);
  }
}
void loop() {
  // Serial_process();
  static double theta_r = 0;
  run_every(20) {
    theta_r += M_PI / 360.0;
    if (theta_r >= 2 * M_PI) theta_r = 0;
    //defines desired x and y postions
    px = 0.015 * cos(theta_r);
    py = 0.015 * sin(theta_r);
    //calculates q1, q2, and q3 given px, py, and theta
    InvKinRRR(px, py, 0);
  }
  MoveServos();
  // run_every(100)
}