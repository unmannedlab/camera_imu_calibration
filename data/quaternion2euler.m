## Copyright (C) 2022 SMISHR30
## 
## This program is free software: you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful, but
## WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see
## <https://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {} {@var{retval} =} quaternion2euler (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: SMISHR30 <smishr30@na1.ford.com@UGC1FHPK7G3>
## Created: 2022-01-29

function [Euler] = quaternion2euler(Quat)

	% Extract the sin of the theta angle
	sth = max(min(2.0*(Quat(1)*Quat(3)-Quat(4)*Quat(2)),1.0),-1.0); % Note: If Quat is valid then this should only trim at most a few eps...

	% Calculate the required Euler angles
	Q3sq = Quat(3)*Quat(3);
	Euler = [atan2(2.0*(Quat(1)*Quat(4)+Quat(2)*Quat(3)),1.0-2.0*(Q3sq+Quat(4)*Quat(4))) asin(sth) atan2(2.0*(Quat(1)*Quat(2)+Quat(3)*Quat(4)),1.0-2.0*(Quat(2)*Quat(2)+Q3sq))];

end
