function M = dcm_z(theta)
M = [
	cos(theta) -sin(theta) 0
	sin(theta) cos(theta) 0
	0 0 1
];
end