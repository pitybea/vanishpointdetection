/*
 *  main.cpp
 *  Hungaraian Assignment Algorithm
 *
 *  Authored by Ryan Rigdon on [May15].
 *  Copyright 2008 M. Ryan Rigdon
 *	mr.rigdon@gmail.com
 *
 */

//This file is part of The Kuhn-Munkres Assignment Algorithm.
//
//The Kuhn-Munkres Assignment Algorithm is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//The Kuhn-Munkres Assignment Algorithm is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with Foobar.  If not, see <http://www.gnu.org/licenses/>.


#include <iostream>
#include <iomanip>
#include "time.h"
#include "munkres.h"
#include <algorithm>


//max and min sizes of the sample matrix
const int min_size = 10;
const int max_size = 50;


//the maximum and minimum weights that can be assigned to a single edge
const int min_weight = 10;
const int max_weight = 50;

//to show the sample set
const bool display_matrix = true;

//for building pseudo-random test cases
//input custom minimum and maximum values for both the size of the matrix
//and the weights contained in it
void test_generator(std::vector< std::vector<int> > &x)
{
	//seed for rand()
	srand(time(NULL));
	
	//random(ish) size of our test case between min_size and max_size inclusive
	const int num_rows = max_size;
	const int num_columns = min_size;
	
	//resize x to match size
	//all values are initialized to -1
	x.resize(num_columns, std::vector <int> (num_rows, -1));
	
	//load weights with random(ish) values between min_weight and wax_weight inclusive
	for (int i = 0; i < num_columns; i++)
	{
		for (int j = 0; j < num_rows; j++)
		{
			x[i][j] = ((rand() % (max_weight-(min_weight-1))) + min_weight);
		}
	}
	
	if (display_matrix)
	{
		//output the starting vector
		for (int i = 0; i < num_columns; i++)
		{
			for (int j = 0; j < num_rows; j++)
			{
				std::cerr << std::setw(3);
				std::cerr << x[i][j] << " ";
			}
			std::cerr << std::endl;
		}
	}
	
	//for performance testing
	std::cerr << "Dimensions: (" << num_rows << ", " << num_columns << ")" << std::endl;
	/*	std::cout << "\nEnter a character then press enter to run algorithm: ";
	 char c;
	 std::cin >> c;
	 std::cout << "Go Time!" << std::endl;
	 */
}
int main (int argc, char * const argv[]) {
	
	//The vector of vectors of integers that we need to pass in
	std::vector< std::vector<int> > x ;
	
	//Build a test case
	test_generator(x);
	
	//actual class instantiation and function calls
	munkres test;
	test.set_diag(false);
	test.load_weights(x);
	int cost;
   int num_rows = std::min(x.size(), x[0].size()), num_columns = std::max(x.size(), x[0].size());
	ordered_pair *p = new ordered_pair[num_rows];
	cost = test.assign(p);
	
	//output size of the matrix and list of matched vertices
	std::cerr << "The ordered pairs are \n";
	for (int i = 0; i < num_rows; i++)
	{
		std::cerr << "(" << p[i].row << ", " << p[i].col << ")" << std::endl;
	}
	std::cerr << "The total cost of this assignment is " << cost << std::endl;
	std::cerr << "Dimensions: (" << num_rows << ", " << num_columns << ")" << std::endl;
	
	delete p;
	return 1;
}
