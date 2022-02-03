// Program to find the line of intersection of two planes
// Input: six points (three points for two planes each)
// Or equation coefficients in the form of ax + by + cz + d = 0
// (for two planes one each)
// Output: 6 x 1 vector with (pt, direction) along the line of intersection

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <pcl/common/intersections.h>

bool isSameLine(Eigen::VectorXd &line1, Eigen::VectorXd &line2) {
	Eigen::Matrix<double, 3, 3> A;
	A << line1(0)-line2(0), line1(1)-line2(1), line1(2)-line2(2),
		 line1(3), line1(4), line1(5),
		 line2(3), line2(4), line2(5);
	
	Eigen::FullPivLU<Eigen::Matrix3d> lu_decomp(A);
	auto rankVal = lu_decomp.rank();

	return rankVal == 1;
}

// Finding coefficients if three points are given
Eigen::Vector4d equationOfPlane(double x1, double y1, double z1,
								double x2, double y2, double z2,
								double x3, double y3, double z3) {
	
	// Finding the direction cosines for the normals
	Eigen::Vector3d lineInPlane1(x2 - x1, y2 - y1, z2 - z1);
	Eigen::Vector3d lineInPlane2(x3 - x1, y3 - y1, z3 - z1);
	Eigen::Vector3d normal = lineInPlane1.cross(lineInPlane2);
	normal.normalize();
	
	// Finding the constant in the lines equation ax + by + cz + d = 0
	Eigen::Vector3d pt0(x1, y1, z1);
	double d = 0 - pt0.dot(normal);

	double a = normal(0), b = normal(1), c = normal(2);
	Eigen::Vector4d v(a, b, c, d);
	return v;
}

// My implementation of plane to plane intersection
bool planeWithPlaneIntersection(Eigen::Vector4d &plane_a, Eigen::Vector4d &plane_b, 
								Eigen::VectorXd &line, double angular_tolerance = 0.1) {
	// Calculating and normalizing normal of both the planes
	Eigen::Vector3d n0(plane_a(0), plane_a(1), plane_a(2));
	Eigen::Vector3d n1(plane_b(0), plane_b(1), plane_b(2));
	n0.normalize();
	n1.normalize();

	// Finding angle between the planes
	// If is almost 1, then the planes are parallel
	// Hence, no intersection
	double dotProduct = n0.dot(n1);
	if(dotProduct > 1 - sin(angular_tolerance)) {
		return false;
	}

	// Direction cosines of line of intersection is cross product
	Eigen::Vector3d line_direction = n0.cross(n1);
	line_direction.normalized();

	// Modifying the argument meant for result
	line.resize(6);
	line(3) = line_direction(0),
	line(4) = line_direction(1),
	line(5) = line_direction(2);

	// Finding the point
	Eigen::Matrix<double, 2, 2> A;
	Eigen::Matrix<double, 2, 1> B, p;
	B << -plane_a(3), -plane_b(3);
	if(line(3) > line(4) && line(3) > line(5)) {
		A << plane_a(1), plane_a(2),
			plane_b(1), plane_b(2);
		p = A.inverse() * B;
		line(0) = 0;
		line(1) = p(0);
		line(2) = p(1);
	} else if(line(4) > line(3) && line(4) > line(5)) {
		A << plane_a(0), plane_a(2),
			plane_b(0), plane_b(2);
		p = A.inverse() * B;
		line(0) = p(0);
		line(1) = 0;
		line(2) = p(1);
	} else {
		A << plane_a(0), plane_a(1),
			plane_b(0), plane_b(1);
		p = A.inverse() * B;
		line(0) = p(0);
		line(1) = p(1);
		line(2) = 0;
	}

	// Intersection exists
	return true;
}

int main() {
	// Variables used in program
	// Points on the plane
	double x00, y00, z00,
			x01, y01, z01,
			x02, y02, z02,
			x10, y10, z10,
			x11, y11, z11,
			x12, y12, z12;
	// Coefficient of plane equation ax+by+cz+d=0
	double a0, b0, c0, d0;
	double a1, b1, c1, d1;
	// Coefficients in vector (4 x 1) form
	Eigen::Vector4d plane0, plane1;
	// bool flag and Coefficients of line (pt, direction) from my method
	Eigen::VectorXd myLine;
	bool myIntersect;
	// bool flag and Coefficients of line (pt, direction) from PCL
	Eigen::VectorXd PCLLine;
	bool PCLIntersect;
	// Checking for test mode
	bool testMode = false;
	std::cout<<"Do you want test from testCases.txt (1 for true, 0 for false)"<<std::endl;
	std::cin>>testMode;
	
	if(testMode) {
		
		// Opening file and checking
		std::fstream file;
		file.open("../testCases.txt", std::ios::in);
		if(!file.is_open()) {
			std::cout<<"Error opening the file"<<std::endl;
			exit(-1);
		}

		// Running for test cases
		int testCases;
		file>>testCases;
		for(int t = 0; t < testCases; t++) {
			bool isEquation = false;
			file>>isEquation;
			if(isEquation) {	// If equation form directly reading a, b, c, and d
				file>>a0>>b0>>c0>>d0
					>>a1>>b1>>c1>>d1;
				plane0 << a0, b0, c0, d0;
				plane1 << a1, b1, c1, d1;
			} else {	// Else directly pts and calculating a, b, c, and d
				file>>x00>>y00>>z00
					>>x01>>y01>>z01
					>>x02>>y02>>z02
					>>x10>>y10>>z10
					>>x11>>y11>>z11
					>>x12>>y12>>z12;
				// Getting equation from points
				plane0 = equationOfPlane(x00, y00, z00, x01, y01, z01, x02, y02, z02);
				plane1 = equationOfPlane(x10, y10, z10, x11, y11, z11, x12, y12, z12);
			}
			myIntersect = planeWithPlaneIntersection(plane0, plane1, myLine);
			PCLIntersect = pcl::planeWithPlaneIntersection(plane0, plane1, PCLLine);
			if(!(myIntersect ^ PCLIntersect) & isSameLine(myLine, PCLLine)) {
				std::cout<<"Test case # "<<t+1<<" passed"<<std::endl;
				std::cout<<myLine.transpose()<<std::endl;
				std::cout<<PCLLine.transpose()<<std::endl;
			} else {
				std::cout<<"Test case # "<<t+1<<" failed"<<std::endl;
			}
		}
	} else {
		// Taking input from the user
		// user input x y z coordinates for six points
		std::cout<<"Please enter the points on the plane 1: Format x y z"<<std::endl;
		std::cout<<"Enter point 1"<<std::endl;
		std::cin>>x00>>y00>>z00;
		std::cout<<"Enter point 2"<<std::endl;
		std::cin>>x01>>y01>>z01;
		std::cout<<"Enter point 3"<<std::endl;
		std::cin>>x02>>y02>>z02;
		std::cout<<"Please enter the points on the plane 2: Format x y z"<<std::endl;
		std::cout<<"Enter point 1"<<std::endl;
		std::cin>>x10>>y10>>z10;
		std::cout<<"Enter point 2"<<std::endl;
		std::cin>>x11>>y11>>z11;
		std::cout<<"Enter point 3"<<std::endl;
		std::cin>>x12>>y12>>z12;
		plane0 = equationOfPlane(x00, y00, z00, x01, y01, z01, x02, y02, z02);
		plane1 = equationOfPlane(x10, y10, z10, x11, y11, z11, x12, y12, z12);
		myIntersect = planeWithPlaneIntersection(plane0, plane1, myLine);
		PCLIntersect = pcl::planeWithPlaneIntersection(plane0, plane1, PCLLine);
		if(!(myIntersect ^ PCLIntersect) & isSameLine(myLine, PCLLine)) {
			std::cout<<"Correct answer"<<std::endl;
			std::cout<<myLine.transpose()<<std::endl;
			std::cout<<PCLLine.transpose()<<std::endl;
		} else {
			std::cout<<"Test case failed"<<std::endl;
		}
	}

	// std::cout<<line_dir<<std::endl;
	// std::cout<<line<<std::endl;

	// Finding the point on the line of intersection
	// if(abs(line_dir.norm()) < 1e-3) {
	// 	std::cout<<"The planes are parallel"<<std::endl;
	// 	std::cout<<"There is no line of intersection"<<std::endl;
	// } else {
	// 	double z = (n1(0)*d0 - n0(0)*d1) / (n1(0)*n0(2) - n0(0)*n1(2));
	// 	double y = 0;
	// 	double x = (d0 - n0(2)*y) / n0(0);

	// 	std::cout<<"The point along the line of intersection is: "<<std::endl;
	// 	std::cout<<"pt = ("<<x<<", "<<y<<", "<<z<<")"<<std::endl;
	// 	std::cout<<"The direction cosine along the line of intersection is: "<<std::endl;
	// 	std::cout<<line_dir<<std::endl;
	// 	std::cout<<"Pcl's answer: "<<std::endl;
	// 	std::cout<<line<<std::endl;
	// }
	return 0;
}