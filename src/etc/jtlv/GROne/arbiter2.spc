LTLSPEC -- Assumptions
	(
		!e.r1 &
		[]((e.r1 != s.g1) -> (e.r1 = next(e.r1))) &
		[]<>(!(e.r1 & s.g1))
	) &
	(
		!e.r2 &
		[]((e.r2 != s.g2) -> (e.r2 = next(e.r2))) &
		[]<>(!(e.r2 & s.g2))
	);

LTLSPEC -- Guarantees
	(([](!(s.g1 & s.g2))) & -- MUX
	
	(!s.g1 & []((e.r1 = s.g1) -> (s.g1 = next(s.g1))) &
		( []<>(e.r1 = s.g1) ) ) &
	
	(!s.g2 & []((e.r2 = s.g2) -> (s.g2 = next(s.g2))) &
			( []<>(e.r2 = s.g2) ) ) )
