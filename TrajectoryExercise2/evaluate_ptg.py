#!/usr/bin/env python

from ptg import PTG
from helpers import Vehicle, show_trajectory

def main():
	vehicle = Vehicle([0,10,0, 0,0,0])
	predictions = {0: vehicle}
	target = 0
	delta = [0, 0, 0, 0, 0 ,0]
	start_s = [10, 10, 0]
	start_d = [4, 0, 0]
	T = 5.0
	best = PTG(start_s, start_d, target, delta, T, predictions)
	show_trajectory(best[0], best[1], best[2], vehicle)

if __name__ == "__main__":
	main()