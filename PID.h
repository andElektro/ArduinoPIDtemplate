#ifndef PID_H
#define PID_H

/*
   PWM frequency ~980Hz ~1ms
   analogWrite(0); //0% duty cycle
   analogWrite(255); //100% duty cycle
*/

/*
   u(n) = u(n-1) + K_p*{e(n)-e(n-1)} + K_i*T_s*e(n) + K_d/T_s*{e(n)-2e(n-1)+e(n-2)}
   T_s = Sampling time of the Analog to Digital (A/D)
   u(n) = Discrete time PID controller output
   e(n) = r(n)-y(n) = Error signal
   r(n) = reference signal
   y(n) = Measured output of process
   n = Discrete interval of time is an integer
   K_p,K_i,K_d = Proportional, Integral, Derivative gain constants respectively.
*/

template<typename U, typename R, typename Y> //U=Discrete PID output, R=Reference, Y=Measured output of process
class PID
{
  private:
    float Kp; //Proportional parameter
    float Ki; //Integral parameter
    float Kd; //Derivative parameter
    float Ts; //Samplingtime
    float KiTs; //Ki*Ts
    float KdTs; //Kd/Ts
    double err = 0; //Optional variable for error sum.
    float e[3] = {0, 0, 0}; //Error = reference - measured output of process => e = r - y
    U u; //Discrete PID output

  public:
    PID(float& _Kp, float& _Ki, float& _Kd, float& _Ts);
    ~PID();
    U get(R& _r, Y& _y);
    //U get(R _r, Y _y); //Use this if pass by value
};

template<typename U, typename R, typename Y>
PID<U, R, Y>::PID(float& _Kp, float& _Ki, float& _Kd, float& _Ts)
  : Kp(_Kp), Ki(_Ki), Kd(_Kd), Ts(_Ts)
{
  KiTs = Ki * Ts;
  KdTs = Kd / Ts;
}

template<typename U, typename R, typename Y>
PID<U, R, Y>::~PID()
{
}

template<typename U, typename R, typename Y>
U PID<U, R, Y>::get(R& _r, Y& _y) {
  e[2] = e[1];
  e[1] = e[0];
  e[0] = _r - _y;
  //err = e[1] + (_r - _y); //When used, replace it with e[0] that is multipied with KiTs.
  u = u + Kp * (e[0] - e[1]) + KiTs * e[0] + KdTs * (e[0] - 2 * e[1] + e[2]);
  return u;
}

//template<typename U, typename R, typename Y>
//U PID<U, R, Y>::get(R _r, Y _y) { //Pass by value function.
//  e[2] = e[1];
//  e[1] = e[0];
//  e[0] = _r - _y;
//  u = u + Kp * (e[0] - e[1]) + KiTs * e[0] + KdTs * (e[0] - 2 * e[1] + e[2]);
//  return u;
//}





#endif
