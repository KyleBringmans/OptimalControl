// updated at: 18-Feb-2020 18:15:42
#include <students/control/Attitude.hpp>

Attitude::State Attitude::codegenNextStateEstimate(State stateEstimate,
                                                   ControlSignal ctrl,
                                                   Measurement meas) {
    // Nextstate = A * stateEstimate + L * (C * StateEstimate - meas) + B * crtl
    
    // C is een identiteitsmatrix 
    // 1) quaternion product: C * stateEstimate - meas: stateEstimateMinusMeas
    State stateEstMinusMeas = State(stateEstimate.q,stateEstimate.w,
                                                        stateEstimate.n);
    stateEstMinusMeas.q = stateEstimate.q - meas.q;
    stateEstMinusMeas.w = stateEstimate.w - meas.w;
    // n component niet nodig want C is daar overal 0

    // 2) A * stateEstimate
    State ATimesStateEstimate = State();
    // #codegen_ATimesStateEstimate
/* ATimesStateEstimate <-- [Ad] * stateEstimate */
ATimesStateEstimate.q.x = (               1) * stateEstimate.q.x+ (      0.00105042) * stateEstimate.w.x+ (     2.99519e-06) * stateEstimate.n.x;
ATimesStateEstimate.q.y = (               1) * stateEstimate.q.y+ (      0.00105042) * stateEstimate.w.y+ (     2.82782e-06) * stateEstimate.n.y;
ATimesStateEstimate.q.z = (               1) * stateEstimate.q.z+ (      0.00105042) * stateEstimate.w.z+ (    -1.80648e-06) * stateEstimate.n.z;
ATimesStateEstimate.w.x = (               1) * stateEstimate.w.x+ (      0.00564637) * stateEstimate.n.x;
ATimesStateEstimate.w.y = (               1) * stateEstimate.w.y+ (      0.00533084) * stateEstimate.n.y;
ATimesStateEstimate.w.z = (               1) * stateEstimate.w.z+ (     -0.00340548) * stateEstimate.n.z;
ATimesStateEstimate.n.x = (        0.941742) * stateEstimate.n.x;
ATimesStateEstimate.n.y = (        0.941742) * stateEstimate.n.y;
ATimesStateEstimate.n.z = (        0.941742) * stateEstimate.n.z;
    
    
    
    // 3) L * stateEstMinusMeas
    State LTimesMinus = State();
    // #codegen_LtimesMinusMeas
/* LTimesMinus <-- [L] * stateEstMinusMeas */
LTimesMinus.q.x = (    -0.000304463) * stateEstMinusMeas.q.x+ (     -0.00106345) * stateEstMinusMeas.w.x;
LTimesMinus.q.y = (    -0.000300113) * stateEstMinusMeas.q.y+ (     -0.00106227) * stateEstMinusMeas.w.y;
LTimesMinus.q.z = (    -0.000304453) * stateEstMinusMeas.q.z+ (     -0.00105244) * stateEstMinusMeas.w.z;
LTimesMinus.w.x = (    -8.60863e-05) * stateEstMinusMeas.q.x+ (      -0.0370162) * stateEstMinusMeas.w.x;
LTimesMinus.w.y = (    -8.36923e-05) * stateEstMinusMeas.q.y+ (      -0.0353284) * stateEstMinusMeas.w.y;
LTimesMinus.w.z = (     -8.6282e-05) * stateEstMinusMeas.q.z+ (      -0.0242975) * stateEstMinusMeas.w.z;
LTimesMinus.n.x = (     3.71172e-06) * stateEstMinusMeas.q.x+ (        -0.10889) * stateEstMinusMeas.w.x;
LTimesMinus.n.y = (     3.44024e-06) * stateEstMinusMeas.q.y+ (       -0.105142) * stateEstMinusMeas.w.y;
LTimesMinus.n.z = (    -2.18707e-06) * stateEstMinusMeas.q.z+ (       0.0782705) * stateEstMinusMeas.w.z;

    // 4) B * ctrl
    State BTimesControl = State();
    // #codegen_BTimesControl
/* BTimesControl <-- [Bd] * ctrl */
BTimesControl.q.x = (     7.01939e-06) * ctrl.x;
BTimesControl.q.y = (     6.62713e-06) * ctrl.y;
BTimesControl.q.z = (     0.000229067) * ctrl.z;
BTimesControl.w.x = (        0.019948) * ctrl.x;
BTimesControl.w.y = (       0.0188333) * ctrl.y;
BTimesControl.w.z = (        0.432173) * ctrl.z;
BTimesControl.n.x = (         6.78998) * ctrl.x;
BTimesControl.n.y = (         6.78998) * ctrl.y;
BTimesControl.n.z = (         6.78998) * ctrl.z;

    // 5) Sumation of components
    State nextState = State();
    nextState.q = ATimesStateEstimate.q + LTimesMinus.q + BTimesControl.q;
    nextState.w = ATimesStateEstimate.w + LTimesMinus.w + BTimesControl.w;
    nextState.n = ATimesStateEstimate.n + LTimesMinus.n + BTimesControl.n;

    return {nextState};
}

Attitude::ControlSignal Attitude::codegenControlSignal(State stateEstimate,
                                                       Integral integralWindup,
                                                       Reference ref) {

    // [x_e u_e] = G * ref
    // ctrl = u_e + K * (stateEstimate - x_e)

    // 1) [x_e u_e] = G * ref
    State StateEqui = State();
    ControlSignal ControlEqui = ControlSignal();
    // #codegen_GTimesRef
/* StateEqui <-- G * ref */
StateEqui.q.x = (               1) * ref.x;
StateEqui.q.y = (               1) * ref.y+ (     5.74265e-25) * 0+ (     1.05876e-26) * 0;
StateEqui.q.z = (               1) * ref.z+ (     9.29137e-24) * 0+ (    -1.12312e-19) * 0;
StateEqui.w.x = (    -5.24278e-35) * ref.x+ (        0.999999) * 0+ (    -4.24728e-22) * 0+ (    -2.07293e-24) * 0;
StateEqui.w.y = (    -2.37582e-39) * ref.x+ (    -6.50226e-22) * 0+ (        0.999999) * 0+ (     4.23524e-22) * 0;
StateEqui.w.z = (      7.6405e-24) * ref.x+ (     2.30414e-21) * 0+ (     2.34495e-22) * 0+ (        0.999999) * 0;
StateEqui.n.x = (     2.67664e-32) * ref.x+ (    -9.48328e-05) * 0+ (     3.77729e-16) * 0+ (     8.49642e-19) * 0;
StateEqui.n.y = (    -5.53724e-35) * ref.x+ (     5.18832e-20) * 0+ (    -0.000100446) * 0+ (    -2.89085e-18) * 0;
StateEqui.n.z = (    -4.59584e-14) * ref.x+ (     1.23616e-14) * 0+ (     2.39007e-19) * 0+ (     -0.00182325) * 0;
ControlEqui.x = (     2.29581e-34) * ref.x+ (    -8.13588e-07) * 0+ (     3.23997e-18) * 0+ (      7.2878e-21) * 0;
ControlEqui.y = (    -4.74971e-37) * ref.x+ (     4.45042e-22) * 0+ (    -8.61753e-07) * 0+ (    -2.47971e-20) * 0;
ControlEqui.z = (    -3.94194e-16) * ref.x+ (     1.06028e-16) * 0+ (     2.05001e-21) * 0+ (    -1.56436e-05) * 0;


    // 2)  stateEstimate - x_e
    State StateEstMinusStateEqui = State();
    StateEstMinusStateEqui.q = stateEstimate.q - stateEqui.q;
    StateEstMinusStateEqui.w = stateEstimate.w - stateEqui.w;
    StateEstMinusStateEqui.n = stateEstimate.n - stateEqui.n;

    // 3) K * StateEstMinusStateEqui
    ControlSignal KTimesState = ControlSignal();
    // #codegen_KTimesState
/* KTimesState <-- K * StateEstMinusStateEqui */
KTimesState.x = (        -139.074) * StateEstMinusStateEqui.q.x+ (        -2.78921) * StateEstMinusStateEqui.w.x+ (        -0.14632) * StateEstMinusStateEqui.n.x;
KTimesState.y = (        -139.292) * StateEstMinusStateEqui.q.y+ (        -2.87077) * StateEstMinusStateEqui.w.y+ (         -0.1461) * StateEstMinusStateEqui.n.y;
KTimesState.z = (        -45.8934) * StateEstMinusStateEqui.q.z+ (        -2.12128) * StateEstMinusStateEqui.w.z+ (     -0.00554159) * StateEstMinusStateEqui.n.z;


    // 4) u_e + KTimesState
    ControlSignal Control = ControlSignal();
    Control = ControlEqui + KTimesState;

    return {Control};
}

Attitude::Integral Attitude::codegenIntegralWindup(State stateEstimate,
                                                   Integral integralWindup,
                                                   Reference ref) {
    // TODO
    (void) stateEstimate;
    (void) integralWindup;
    (void) ref;
    return {};
}
