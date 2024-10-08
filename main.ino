
#include "config.h"
#include "ESP8266_ISR_Servo.h"
#include <Eigen/Dense>
int servoIndex[3];
double px = 0.0;
double py = 0.0;
double theta = 0.0;
int out_of_reach;
bool ff;
uint16_t microsec;
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
    uint16_t microsec_now = interpolate(deg_to_rad(p[servo]), theta_min[servo], theta_max[servo], m_of_theta_min[servo], m_of_theta_max[servo]);  //interpolates
    // if (servo == 1) PRINT_SERIAL_COMMA(p[servo], microsec_now);
    // myservo[servo].writeMicroseconds((int)microsec_now);
    ISR_Servo.setPulseWidth(servoIndex[servo],microsec_now);
    // pwm.setPWM(servo, 0, cvt_angle(p[servo]));
    //Serial.println("servo[" + String(servo) + "]:" + String(p[servo]));
  }
}
void ForwardRRR(double q0, double q1, double q2, double theta) {

    // Tính toán tọa độ e1, e2, e3
    double e1x = b1x + L1 * cos(q0);
    double e1y = b1y + L1 * sin(q0);

    double e2x = b2x + L1 * cos(q1);
    double e2y = b2y + L1 * sin(q1);

    double e3x = b3x + L1 * cos(q2);
    double e3y = b3y + L1 * sin(q2);

    // Tính các cos và sin của theta và a_angles
    double cos_1 = rp[0] * cos(theta + a_angles[0]);
    double cos_2 = rp[0] * cos(theta + a_angles[1]);
    double cos_3 = rp[0] * cos(theta + a_angles[2]);

    double sin_1 = rp[0] * sin(theta + a_angles[0]);
    double sin_2 = rp[0] * sin(theta + a_angles[1]);
    double sin_3 = rp[0] * sin(theta + a_angles[2]);

    // Tính các giá trị G1, G2, G3, G4, G5, G6
    double G1 = 2 * cos_2 - 2 * e2x - 2 * cos_1 + 2 * e1x;
    double G2 = 2 * e1y - 2 * e2y + 2 * sin_2 - 2 * sin_1;
    double G3 = (e2x * e2x + e2y * e2y) - (e1x * e1x + e1y * e1y) + (2 * e1x * cos_1 - 2 * e2x * cos_2) + (2 * e1y * sin_1 - 2 * e2y * sin_2);

    double G4 = (2 * e1x) - (2 * e3x) + (2 * cos_3) - (2 * cos_1);
    double G5 = (2 * e1y) - (2 * e3y) + (2 * sin_3) - (2 * sin_1);
    double G6 = (e3x * e3x + e3y * e3y) - (e1x * e1x + e1y * e1y) + (2 * e1x * cos_1 - 2 * e3x * cos_3) + (2 * e1y * sin_1 - 2 * e3y * sin_3);

    // Ma trận V và vector W
    Matrix2d V;
    V(0, 0) = G1;
    V(0, 1) = G2;
    V(1, 0) = G4;
    V(1, 1) = G5;

    Vector2d W;
    W(0) = -G3;
    W(1) = -G6;

    // Giải hệ phương trình V * F = W để tìm F
    Vector2d F = V.colPivHouseholderQr().solve(W);

    double cx = F(0);
    double cy = F(1);
}
void InvKinRRR(double px, double py, double theta) {


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
      // Serial.print("coord:");
      PRINT_SERIAL_COMMA(result.x, result.y, result.theta);
      // Serial.print("angle:");
      InvKinRRR(result.x, result.y, deg_to_rad(result.theta));
      // PRINT_SERIAL_COMMA(target[0], target[1], target[2]);
      // int sv_ind = c.substring(0, index_now).toInt();
      // angle = c.substring(index_now + 1, index_cal).toFloat();
      // microsec = interpolate(deg_to_rad(angle), theta_min[sv_ind], theta_max[sv_ind], m_of_theta_min[sv_ind], m_of_theta_max[sv_ind]);  //interpolates
      // ISR_Servo.setPulseWidth(servoIndex[sv_ind],microsec);
    }
  }
  return result;
}
void setup() {
  Serial.begin(57600);
  // myservo[0].attach(D4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  // myservo[1].attach(D5, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  // myservo[2].attach(D6, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  
  for (int i = 0; i < 3; i++) {
    servoIndex[i] = ISR_Servo.setupServo(servo_pin[i], MIN_PULSE, MAX_PULSE);
    uint16_t microsec_now = cvt_to_micro(InitialPosition[i], i);  //interpolates  '
    // myservo[i].writeMicroseconds(microsec_now);
    ISR_Servo.setPulseWidth(servoIndex[i],microsec_now);
    p[i] = InitialPosition[i];
    target[i] = InitialPosition[i];
    home_offset_angle[i] = origin_servo_angle[i] - deg_to_rad(InitialPosition[i]);
  }
  Serial.setTimeout(10);
}
void loop() {
  // Serial_process();
  static double theta_r = 0;
  // run_every(20) {
  //   theta_r += M_PI / 1440.0;
  //   if (theta_r >= 2 * M_PI) theta_r = 0;
  //   //defines desired x and y postions
  //   px = 0.015 * cos(theta_r);
  //   py = 0.015 * sin(theta_r);
  //   //calculates q1, q2, and q3 given px, py, and theta
  //   InvKinRRR(px, py, 0);
  // }
  unsigned long time_ = millis();
  int m[3];


  //   Serial.println(theta_r);

  //   //determines microsecond delays from q1, q2, and q3
  // }

    MoveServos();
  // delay(1);
  // run_every(100) PRINT_SERIAL_COMMA(angle,microsec);
}