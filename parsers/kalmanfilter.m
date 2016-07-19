% Kalman Filter Implementation
%
% USAGE:
%
% s = kalmanfilter(s)
%
% "s" is a "system" struct containing various fields used as input
% and output. The state estimate "x" and its covariance "P" are
% updated by the function. The other fields describe the mechanics
% of the system and are left unchanged. A calling routine may change
% these other fields as needed if state dynamics are time-dependent;
% otherwise, they should be left alone after initial values are set.
% The exceptions are the observation vector "z" and the input control
% (or forcing function) "u." If there is an input function, then
% "u" should be set to some nonzero value by the calling routine.
%
% SYSTEM DYNAMICS:
%
% The system evolves according to the following difference equations,
% where quantities are further defined below:
%
% x = Ax + Bu + w  meaning the state vector x evolves during one time
%                  step by premultiplying by the "state transition
%                  matrix" A. There is optionally (if nonzero) an input
%                  vector u which affects the state linearly, and this
%                  linear effect on the state is represented by
%                  premultiplying by the "input matrix" B. There is also
%                  gaussian process noise w.
% z = Hx + v       meaning the observation vector z is a linear function
%                  of the state vector, and this linear relationship is
%                  represented by premultiplication by "observation
%                  matrix" H. There is also gaussian measurement
%                  noise v.
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
%
% VECTOR VARIABLES:
%
% s.x = state vector estimate. In the input struct, this is the
%       "a priori" state estimate (prior to the addition of the
%       information from the new observation). In the output struct,
%       this is the "a posteriori" state estimate (after the new
%       measurement information is included).
% s.z = observation vector
% s.u = input control vector, optional (defaults to zero).
%
% MATRIX VARIABLES:
%
% s.A = state transition matrix (defaults to identity).
% s.P = covariance of the state vector estimate. In the input struct,
%       this is "a priori," and in the output it is "a posteriori."
%       (required unless autoinitializing as described below).
% s.B = input matrix, optional (defaults to zero).
% s.Q = process noise covariance (defaults to zero).
% s.R = measurement noise covariance (required).
% s.H = observation matrix (defaults to identity).
%
% NORMAL OPERATION:
%
% (1) define all state definition fields: A,B,H,Q,R
% (2) define intial state estimate: x,P
% (3) obtain observation and control vectors: z,u
% (4) call the filter to obtain updated state estimate: x,P
% (5) return to step (3) and repeat
%
% INITIALIZATION:
%
% If an initial state estimate is unavailable, it can be obtained
% from the first observation as follows, provided that there are the
% same number of observable variables as state variables. This "auto-
% intitialization" is done automatically if s.x is absent or NaN.
%
% x = inv(H)*z
% P = inv(H)*R*inv(H')
%
% This is mathematically equivalent to setting the initial state estimate
% covariance to infinity.

function s = kalmanfilter(s)

    % set defaults for absent fields:
    if ~isfield(s,'x'); s.x=nan*z; end
    if ~isfield(s,'P'); s.P=nan; end
    if ~isfield(s,'z'); error('Observation vector missing'); end
    if ~isfield(s,'u'); s.u=0; end
    if ~isfield(s,'A'); s.A=eye(length(x)); end
    if ~isfield(s,'B'); s.B=0; end
    if ~isfield(s,'Q'); s.Q=zeros(length(x)); end
    if ~isfield(s,'R'); error('Observation covariance missing'); end
    if ~isfield(s,'H'); s.H=eye(length(x)); end

    if isnan(s.x)
       % initialize state estimate from first observation
       if diff(size(s.H))
          error('Observation matrix must be square and invertible for state autointialization.');
       end
       s.x = inv(s.H)*s.z;
       s.P = inv(s.H)*s.R*inv(s.H'); 
    else

       % This is the code which implements the discrete Kalman filter:

       % Prediction for state vector and covariance:
       s.x = s.A*s.x + s.B*s.u;
       s.P = s.A * s.P * s.A' + s.Q;

       % Compute Kalman gain factor:
       K = s.P*s.H'*inv(s.H*s.P*s.H'+s.R);

       % Correction based on observation:
       s.x = s.x + K*(s.z-s.H*s.x);
       s.P = s.P - K*s.H*s.P;

    end

end


  
