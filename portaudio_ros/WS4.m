randNum = rand*200-100;
choice = menu ('Choose your type of random number', 'ceil', 'round', 'sign');
switch choice
case 1
disp(ceil(randNum));
case 2
disp(round(randNum));
case 3
disp(sign(randNum));
end