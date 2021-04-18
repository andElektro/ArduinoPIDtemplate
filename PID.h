#ifndef PID_H
#define PID_H

/*
   Arduino:
   PWM frequency ~980Hz ~1ms
   analogWrite(0); //0% duty cycle
   analogWrite(255); //100% duty cycle
*/

/*
   (1: first) u(n) = u(n-1) + Kp*{e(n)-e(n-1)} + Ti*Ts*e(n) + Td/Ts*{e(n)-2e(n-1)+e(n-2)}
   (1: second) w(n) = w(n-1) + e(n) , u(n) = Kp{e(n) + Ts/Ti*{w(n)} + Td/Ts*{e(n)-e(n-1)}}
   w(n) = sum of errors
   Ts = Sampling time of the Analog to Digital (A/D)
   u(n) = Discrete time PID controller output
   e(n) = r(n)-y(n) = Error signal
   r(n) = reference signal
   y(n) = Measured output of process
   n = Discrete interval of time is an integer
   Kp, Ti, Td = Proportional, Integral, Derivative gain constants respectively.
*/

template<typename U, typename R, typename Y> //U=Discrete PID output, R=Reference, Y=Measured output of process
class PID
{
  private:
    float Kp; //Proportional parameter
    float Ti; //Integral parameter
    float Td; //Derivative parameter
    float Ts; //Samplingtime
    float TiTs; //Ti*Ts
    float TdTs; //Td/Ts
    float TsTi; //Ts/Ti
    float ef[3] = {0, 0, 0}; //First Error = reference - measured output of process => e = r - y
    double w[2] = {0, 0}; //Sum of errors
    float es[2] = {0, 0}; //Second Error = reference - measured output of process => e = r - y
    U u; //Discrete PID output

  public:
    PID(float& _Kp, float& _Ti, float& _Td, float& _Ts);
    ~PID();
    U firstRefGet(R& _r, Y& _y);
    U firstValGet(R _r, Y _y);
    U secondRefGet(R& _r, Y& _y);
    U SecondValGet(R _r, Y _y);
};

template<typename U, typename R, typename Y>
PID<U, R, Y>::PID(float& _Kp, float& _Ti, float& _Td, float& _Ts)
  : Kp(_Kp), Ti(_Ti), Td(_Td), Ts(_Ts)
{
  TiTs = Ti * Ts;
  TdTs = Td / Ts;
  TsTi = Ts / Ti;
}

template<typename U, typename R, typename Y>
PID<U, R, Y>::~PID()
{
}

template<typename U, typename R, typename Y>
U PID<U, R, Y>::firstRefGet(R& _r, Y& _y) {
  ef[2] = ef[1];
  ef[1] = ef[0];
  ef[0] = _r - _y;
  u = u + Kp * (ef[0] - ef[1]) + TiTs * ef[0] + TdTs * (ef[0] - 2 * ef[1] + ef[2]);
  return u;
}

template<typename U, typename R, typename Y>
U PID<U, R, Y>::firstValGet(R _r, Y _y) {
  ef[2] = ef[1];
  ef[1] = ef[0];
  ef[0] = _r - _y;
  u = u + Kp * (ef[0] - ef[1]) + TiTs * ef[0] + TdTs * (ef[0] - 2 * ef[1] + ef[2]);
  return u;
}

template<typename U, typename R, typename Y>
U PID<U, R, Y>::secondRefGet(R& _r, Y& _y) {
  es[1] = es[0];
  es[0] = _r - _y;
  w[1] = w[0];
  w[0] = w[1] + es[0];
  u = Kp * (es[0] + TsTi*es[0] + TdTs*(es[0] - es[1]));
  return u;
}

template<typename U, typename R, typename Y>
U PID<U, R, Y>::SecondValGet(R _r, Y _y) {
  es[1] = es[0];
  es[0] = _r - _y;
  w[1] = w[0];
  w[0] = w[1] + es[0];
  u = Kp * (es[0] + TsTi*es[0] + TdTs*(es[0] - es[1]));
  return u;
}

#endif
