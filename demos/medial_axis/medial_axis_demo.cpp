//
// Created by steven on 12/21/23.
//

#include "medial_axis_demo.h"
#include <QApplication>

MedialAxisDemo::MedialAxisDemo() {
	setWindowTitle("Medial Axis");
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	MedialAxisDemo demo;
	demo.show();
	app.exec();
}