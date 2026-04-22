#include <iostream>
#include <fstream>
#include "DisplayManager.h"
#include "Hydrodynamics.hpp"
#include "VehicleInfo.hpp"
#include "PWMtoForce.hpp"
#include "Tools.hpp"
#include "State.hpp"
#include <chrono>

using namespace VehicleInfo;
using namespace Math;
using Clock = std::chrono::steady_clock;

void f(State* s, float* pwm);				// replaces the value in s with its derivative
void csv(std::ofstream& file, glm::vec3 pos, glm::vec3 vel, int step);        // store position and velocity in a csv file

int main() {

	DisplayManager myWindow;
	myWindow.createDisplay();
	int i = 0;
	State s{glm::vec3(0, 0, 0),        // position
		 glm::vec3(0, 0, 0),		   // velocity
		  glm::quat(1, 0, 0, 0),	   // orientation
		   glm::vec3(0, 0, 0)};	       // angular velocity
	float pwm[8] = {1500, 1500, 1500, 1500, 1600, 1400, 1600, 1400};

	std::ofstream file("pose.csv");

	while(!myWindow.shouldClose()) {

		myWindow.updateDisplay();
		csv(file, s.position, s.velocity, i);
		std::cout<<"seconds: "<<(0.01f*i)<<std::endl;
		std::cout<<"xyz: "<<s.position.x<<","<<s.position.y<<","<<s.position.z<<std::endl;
		std::cout<<"uvw: "<<s.velocity.x<<","<<s.velocity.y<<","<<s.velocity.z<<std::endl;
		glm::vec3 rpy = glm::eulerAngles(s.orientation);
		std::cout<<"rpy: "<<rpy.x<<","<<rpy.y<<","<<rpy.z<<std::endl;
		std::cout<<"pqr: "<<s.angular_velocity.x<<","<<s.angular_velocity.y<<","<<s.angular_velocity.z<<std::endl;
		// glm::quat q = glm::normalize(glm::quat(1,0,0,0));
		// std::cout<<q.w<<","<<q.x<<","<<q.y<<","<<q.z<<std::endl;
		std::cout<<"-------------------------------------------------"<<std::endl;
		Tools::Runge_Kutta4(&s, f, pwm);
		i++;
		
	}
	myWindow.closeDisplay();
	return 0;
}

void csv(std::ofstream& file, glm::vec3 pos, glm::vec3 vel, int step) {
	
	file << 0.01f*step << ","
		 << pos.x << ","
		 << pos.y << ","
		 << pos.z << ","
		 << vel.x << ","
		 << vel.y << ","
		 << vel.z
		 << "\n";
}

void f(State* s, float* pwm) {
	// Calculate forces and torques. Input v and w in body frame. Use glm overload for (q^-1)vq to convert from world frame to body frame

	PWMtoForce pwmtoforce;
	pwmtoforce.populateThrusterForces(pwm);

	glm::vec3 force = pwmtoforce.calculateNetForce();
	// std::cout<<force.x<<","<<force.y<<","<<force.z<<std::endl;
	// glm::vec3 test = s->velocity * s->orientation;
	// std::cout<<s->velocity.x<<","<<s->velocity.y<<","<<s->velocity.z<<" | "<<s->orientation.w<<","<<s->orientation.x<<","<<s->orientation.y<<","<<s->orientation.z<<" | "<<test.x<<","<<test.y<<","<<test.z<<std::endl;
	force += Hydrodynamics::calculateLinearDrag(s->velocity * s->orientation);
	// std::cout<<force.x<<","<<force.y<<","<<force.z<<std::endl;
	force += Hydrodynamics::calculateQuadraticDrag(s->velocity * s->orientation);
	// std::cout<<force.x<<","<<force.y<<","<<force.z<<std::endl;
	force += Hydrodynamics::calculateCorriolisA(s->velocity * s->orientation, s->angular_velocity * s->orientation, false);
	/// std::cout<<force.x<<","<<force.y<<","<<force.z<<std::endl;
	force += Hydrodynamics::calculateCorriolisRB(s->velocity * s->orientation, s->angular_velocity * s->orientation, false);
	// std::cout<<force.x<<","<<force.y<<","<<force.z<<std::endl;
	force += glm::vec3(0, 0, VEHICLE_MASS * 9.81) * s->orientation;        // gravity in body frame. Using glm overload for (q^-1)vq
	// std::cout<<force.x<<","<<force.y<<","<<force.z<<std::endl;
	force += glm::vec3(0, 0, -WATER_DENSITY * VOLUME_DISPLACED_WATER * 9.81) * s->orientation;        // bouyancy in body frame. Using glm overload for (q^-1)vq
	// std::cout<<force.x<<","<<force.y<<","<<force.z<<std::endl;

	glm::vec3 torque = pwmtoforce.calculateNetTorque();
	// std::cout<<torque.x<<","<<torque.y<<","<<torque.z<<std::endl;
	torque += Hydrodynamics::calculateLinearDrag(s->angular_velocity * s->orientation);
	torque += Hydrodynamics::calculateQuadraticDrag(s->angular_velocity * s->orientation);
	torque += Hydrodynamics::calculateCorriolisA(s->velocity * s->orientation, s->angular_velocity * s->orientation, true);
	torque += Hydrodynamics::calculateCorriolisRB(s->velocity * s->orientation, s->angular_velocity * s->orientation, true);
	// std::cout<<force.x<<","<<force.y<<","<<force.z<<" | "<<torque.x<<","<<torque.y<<","<<torque.z<<std::endl;

	s->position = s->velocity;
	glm::vec3 totalMass_linear = glm::vec3(VEHICLE_MASS + xDotU, VEHICLE_MASS + yDotV, VEHICLE_MASS + zDotW);
	s->velocity = s->orientation * (force / totalMass_linear);        // convert accel_body to accel_world. Using glm overload for qv(q^-1)

	glm::quat orientationDot = 0.5f * (glm::quat(0, s->angular_velocity.x, s->angular_velocity.y, s->angular_velocity.z) * s->orientation);        // qDot = 1/2 * w_world * q

	glm::vec3 totalInertia = glm::vec3(VEHICLE_Ixx + kDotP, VEHICLE_Iyy + mDotQ, VEHICLE_Izz + nDotR);
	glm::vec3 H = totalInertia * (s->angular_velocity * s->orientation);        // calculate H_body. Using glm overload for (q^-1)vq
	glm::vec3 angular_velocityDot = (torque - glm::cross(s->angular_velocity * s->orientation, H)) / totalInertia;        // calculate wDot in body frame
	s->angular_velocity = s->orientation * angular_velocityDot;        // convert back to world frame and store

	s->orientation = orientationDot;
	// std::cout<<s->orientation.w<<","<<s->orientation.x<<","<<s->orientation.y<<","<<s->orientation.z<<std::endl;
}