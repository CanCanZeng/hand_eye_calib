% handEye - performs hand/eye calibration
% 
%     gHc = handEye(bHg, wHc)
% 
%     bHg - pose of gripper relative to the robot base..
%           (Gripper center is at: g0 = Hbg * [0;0;0;1] )
%           Matrix dimensions are 4x4xM, where M is ..
%           .. number of camera positions. 
%           Algorithm gives a non-singular solution when ..
%           .. at least 3 positions are given
%           Hbg(:,:,i) is i-th homogeneous transformation matrix
%     cHw - extrinsic of camera(world relative to the camera)
%           Dimension: size(Hwc) = size(Hbg)
%     gHc - 4x4 homogeneous transformation from gripper to camera      
%           , that is the camera position relative to the gripper.
%           Focal point of the camera is positioned, ..
%           .. relative to the gripper, at
%                 f = gHc*[0;0;0;1];
%           
% References: R.Tsai, R.K.Lenz "A new Technique for Fully Autonomous 
%           and Efficient 3D Robotics Hand/Eye calibration", IEEE 
%           trans. on robotics and Automaion, Vol.5, No.3, June 1989
%
% % % % % Notation: wHc - pose of camera frame (c) in the world (w) coordinate system
% % % % %                 .. If a point coordinates in camera frame (cP) are known
% % % % %                 ..     wP = wHc * cP
% % % % %                 .. we get the point coordinates (wP) in world coord.sys.
% % % % %                 .. Also refered to as transformation from camera to world
%

function gHc = handEye(bHg, cHw)

M = size(bHg,3);

K = (M*M-M)/2;               % Number of unique camera position pairs
A = zeros(3*K,3);            % will store: skew(Pgij+Pcij)
B = zeros(3*K,1);            % will store: Pcij - Pgij
k = 0;

% Now convert from wHc notation to Hc notation used in Tsai paper.
Hg = bHg;
Hc= cHw; 
% Hc = zeros(4,4,M); 
% for i = 1:M
% %     Hc(:,:,i) = inv(wHc(:,:,i)); 
%     
% end

for i = 1:M
   for j = i+1:M
		Hgij = (Hg(:,:,j))\Hg(:,:,i);    % Transformation from i-th to j-th gripper pose
		Pgij = 2*rot2quat(Hgij);            % ... and the corresponding quaternion
      
		Hcij = Hc(:,:,j)/(Hc(:,:,i));    % Transformation from i-th to j-th camera pose
		Pcij = 2*rot2quat(Hcij);            % ... and the corresponding quaternion

      k = k+1;                            % Form linear system of equations
      A((3*k-3)+(1:3), 1:3) = skew(Pgij+Pcij); % left-hand side
      B((3*k-3)+(1:3))      = Pcij - Pgij;     % right-hand side
      
   end
end

% Rotation from camera to gripper is obtained from the set of equations:
%    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij
% Gripper with camera is first moved to M different poses, then the gripper
% .. and camera poses are obtained for all poses. The above equation uses
% .. invariances present between each pair of i-th and j-th pose.

Pcg_ = A \ B;                % Solve the equation A*Pcg_ = B

% Obtained non-unit quaternin is scaled back to unit value that
% .. designates camera-gripper rotation
Pcg = 2 * Pcg_ / sqrt(1 + Pcg_'*Pcg_);

Rcg = quat2rot(Pcg/2);         % Rotation matrix


% Calculate translational component
k = 0;
for i = 1:M
   for j = i+1:M
		Hgij = (Hg(:,:,j))\Hg(:,:,i);    % Transformation from i-th to j-th gripper pose
		Hcij = Hc(:,:,j)/(Hc(:,:,i));    % Transformation from i-th to j-th camera pose

      k = k+1;                            % Form linear system of equations
      A((3*k-3)+(1:3), 1:3) = Hgij(1:3,1:3)-eye(3); % left-hand side
      B((3*k-3)+(1:3))      = Rcg(1:3,1:3)*Hcij(1:3,4) - Hgij(1:3,4);     % right-hand side
      
   end
end

Tcg = A \ B;

gHc = transl(Tcg) * Rcg;	% incorporate translation with rotation


return


% quat2rot - a unit quaternion(3x1) to converts a rotation matrix (3x3) 
%
%    R = quat2rot(q)
% 
%    q - 3x1 unit quaternion
%    R - 4x4 homogeneous rotation matrix (translation component is zero) 
%        q = sin(theta/2) * v
%        teta - rotation angle
%        v    - unit rotation axis, |v| = 1
%
% See also: rot2quat, rotx, roty, rotz, transl, rotvec

function R = quat2rot(q)

	p = q'*q;
   if( p > 1 )
      disp('Warning: quat2rot: quaternion greater than 1');
   end
   w = sqrt(1 - p);                   % w = cos(theta/2)
   
   R = eye(4);
   R(1:3,1:3) = 2*(q*q') + 2*w*skew(q) + eye(3) - 2*diag([p p p]);
   
return


% rot2quat - converts a rotation matrix (3x3) to a unit quaternion(3x1)
%
%    q = rot2quat(R)
% 
%    R - 3x3 rotation matrix, or 4x4 homogeneous matrix 
%    q - 3x1 unit quaternion
%        q = sin(theta/2) * v
%        teta - rotation angle
%        v    - unit rotation axis, |v| = 1
%
%    
% See also: quat2rot, rotx, roty, rotz, transl, rotvec

function q = rot2quat(R)

	w4 = 2*sqrt( 1 + trace(R(1:3,1:3)) ); % can this be imaginary?
	q = [
		( R(3,2) - R(2,3) ) / w4;
		( R(1,3) - R(3,1) ) / w4;
   	( R(2,1) - R(1,2) ) / w4;
   ];
   
return



%ISHOMOG	test if argument is a homogeneous transformation (4x4)

function h = ishomog(tr)
	h = all(size(tr) == [4 4]);



% skew - returns skew matrix of a 3x1 vector. 
%        cross(V,U) = skew(V)*U
%
%    S = skew(V)
%
%          0  -Vz  Vy
%    S =   Vz  0  -Vx  
%         -Vy  Vx  0
%
% See also: cross



function S = skew(V)
	S = [
   	    0    -V(3)    V(2)
        V(3)      0    -V(1)
       -V(2)    V(1)      0
   ];
return


%TRANSL	Translational transform
%
%	T= TRANSL(X, Y, Z)
%	T= TRANSL( [X Y Z] )
%
%	[X Y Z]' = TRANSL(T)
%
%	[X Y Z] = TRANSL(TG)
%
%	Returns a homogeneous transformation representing a 
%	translation of X, Y and Z.
%
%	The third form returns the translational part of a
%	homogenous transform as a 3-element column vector.
%
%	The fourth form returns a  matrix of the X, Y and Z elements
%	extracted from a Cartesian trajectory matrix TG.
%
%	See also ROTX, ROTY, ROTZ, ROTVEC.

% 	Copyright (C) Peter Corke 1990
function r = transl(x, y, z)
	if nargin == 1
		if ishomog(x)
			r = x(1:3,4);
		elseif size(x, 2) == 16
			r = x(:,13:15);
		else
			t = x(1:3);
			r =    [eye(3)			t;
				0	0	0	1];
		end
	elseif nargin == 3
		t = [x; y; z];
		r =    [eye(3)			t;
			0	0	0	1];
	end

