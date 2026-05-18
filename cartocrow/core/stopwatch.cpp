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
*/

#include "stopwatch.h"

#include <iostream>
#include <string>
#include <chrono>

namespace cartocrow {

using TimeResolution = std::chrono::milliseconds;
std::vector<Stopwatch> watches;

int countDigits(long val) {
	// NB: assume val >= 0
	return val < 10 ? 1 : (1 + countDigits(val / 10));
}

void printLeftAlign(std::string s, int w) {
	std::cout << s;
	w -= s.length();
	while (w > 0) {
		std::cout << " ";
		w--;
	}
}
void printRightAlign(std::string s, int w) {
	w -= s.length();
	while (w > 0) {
		std::cout << " ";
		w--;
	}
	std::cout << s;
}

Stopwatch& Stopwatch::start() {
	start_time = std::chrono::duration_cast<TimeResolution>(
	                 std::chrono::system_clock::now().time_since_epoch())
	                 .count();
	return *this;
}

long Stopwatch::stop() {
	long end_time = std::chrono::duration_cast<TimeResolution>(
	                    std::chrono::system_clock::now().time_since_epoch())
	                    .count();
	count++;
	long duration = end_time - start_time;
	total += duration;
	return duration;
}

long Stopwatch::getTotalTime() {
	return total;
}

long Stopwatch::getCount() {
	return count;
}

Stopwatch& Stopwatch::get(std::string name) {
	for (Stopwatch& w : watches) {
		if (w.name == name) {
			return w;
		}
	}
	Stopwatch w(name);
	watches.push_back(w);
	return watches[watches.size()-1];
}

void Stopwatch::clear() {
	watches.clear();
}

void Stopwatch::printAndClear() {
	Stopwatch::printAll();
	Stopwatch::clear();
}

void Stopwatch::printAll() {
	std::cout << "------ TIMINGS -----------------------------" << std::endl;

	int longestname = 4;
	int longesttime = 5;
	int longestcount = 5;
	for (Stopwatch& w : watches) {
		longestname = std::max(longestname, (int) w.name.length());
		longesttime = std::max(longesttime, countDigits(w.total));
		longestcount = std::max(longestcount, countDigits(w.count));
	}
	int longestaverage = std::max(longesttime, 7);

	printLeftAlign("NAME", longestname);
	std::cout << "  ";
	printLeftAlign("TOTAL", longesttime);
	std::cout << "  ";
	printLeftAlign("COUNT", longestcount);
	std::cout << "  ";
	printLeftAlign("AVERAGE", longestaverage);
	std::cout << "  ";

	for (Stopwatch& w : watches) {
		printLeftAlign(w.name, longestname);
		std::cout << "  ";
		printRightAlign(std::to_string(w.total), longesttime);
		std::cout << "  ";
		printRightAlign(std::to_string(w.count), longestcount);
		std::cout << "  ";
		if (w.count == 0) {
			printRightAlign("-", longestaverage);
		} else {
			printRightAlign(std::to_string(w.total / w.count), longestaverage);
		}
		std::cout << std::endl;
	}
	std::cout << "--------------------------------------------" << std::endl;
}


} // namespace cartocrow