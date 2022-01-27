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
## @deftypefn {} {@var{retval} =} rotm2eul (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: SMISHR30 <smishr30@na1.ford.com@UGC1FHPK7G3>
## Created: 2022-01-26

function [Euler] = rotm2eul(Rotmat)

	% Extract the sin of the theta angle
	sth = max(min(-Rotmat(3,1),1.0),-1.0); % Note: If Rotmat is valid then this should only trim at most a few eps...

	% Calculate the required Euler angles
	Euler = [atan2(Rotmat(2,1),Rotmat(1,1)) asin(sth) atan2(Rotmat(3,2),Rotmat(3,3))];

end
