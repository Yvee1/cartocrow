/*
Copyright (C) 2026  TU Eindhoven

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/                                                                                                \

#pragma once

#include <string>

namespace cartocrow {
		
/// A simple Stopwatch class that allows for tracking wallclock time. Stopwatches are identified by name and use millisecond resolution.
class Stopwatch {

  private:
	std::string name;
	long total = 0;
	long count = 0;
	long start_time = 0;

	Stopwatch(std::string name) : name(name) {}

  public:
	Stopwatch& start();
	long stop();

	long getTotalTime();
	long getCount();

	static Stopwatch& get(std::string name);

	static void clear();

	static void printAndClear();

	static void printAll();
};

}