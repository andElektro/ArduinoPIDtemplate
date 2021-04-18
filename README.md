# ArduinoPIDtemplate
An Arduino template for a PID regulator. 

Not tested IRL.

There are two PID regulators that can be used. Each has a different approach to calculate PID output.

If samplingtime is much faster than the process, one way of determine K, Ti and Td is the Ziegler-Nichols method:

P regulator: K = 0.5*K0

PI regulator: K = 0.45*K0 , Ti = 0.85*T0

PID regulator: K = 0.6*K0 , Ti = 0.5*T0 , Td = 0.125*T0

K0 = Value of K when Ti=0 and Td=0 and the process oscillates.
T0 = Priod of K0.

Karl Johan Åström and Tore Hägglund parameters:

PID regulator: K = 0.35*K0 , Ti = 0.77*T0 , Td = 0.19*T0

