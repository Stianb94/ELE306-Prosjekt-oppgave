
hold on
Omraade = zeros(250,550);
lp = Lattice(Omraade,'grid',50,'root',[50 50]); % create navigation object
lp.plan('cost',[1 50 50]) % create roadmaps
lp.query( [50 50 0], [500 150 pi] ) % find path
lp.plot();
lp.query( [500 150 pi], [100 250 0] )
lp.plot(); % plot the path
