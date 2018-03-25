/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang / Raul Davila
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

	// define number of particles
	num_particles = 100;

	// local particle and random engine
	Particle particle;
	default_random_engine gen;

	// GPS uncertainties (x, y in meters. theta in radians)
	// Only used in initialization (simulator deviations used in prediction step)
	std_GPS_x = 2;
	std_GPS_y = 2;
	std_GPS_theta = 0.2;

	// normal distribution for each coordinate
	normal_distribution<double> dist_x(x, std_GPS_x);
	normal_distribution<double> dist_y(y, std_GPS_y);
	normal_distribution<double> dist_theta(theta, std_GPS_theta);

	for (int i=0;i<num_particles;i++){

		// initialize particles with Gaussian distribution
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;
		particle.id = i;

		// add initialized particle to vector
		particles.push_back(particle);

		// add initialized weight to vector (for resampling step)
		weights.push_back(particle.weight);

		// set initialized flag to true
		is_initialized = true;
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	// local coordinates and random engine
	double x, y, theta;
	default_random_engine gen;

	// avoid division by zero during prediction
	if(yaw_rate<0.00001 && yaw_rate>-0.00001){
		if (yaw_rate>=0){
			yaw_rate = 0.00001;
		}
		else{
			yaw_rate = -0.00001;
		}
	}

	for (int i=0;i<num_particles;i++){

		// predict state according to estimated velocity and yaw rate
		x = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
		y = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
		theta = particles[i].theta + yaw_rate*delta_t;

		// normal distribution for each coordinate
		normal_distribution<double> dist_x(x, std_pos[0]);
		normal_distribution<double> dist_y(y, std_pos[1]);
		normal_distribution<double> dist_theta(theta, std_pos[2]);

		// apply Gaussian noise
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);

	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	// x and y map coordinates for single particle observations
	vector<double> x_map_obs;
	vector<double> y_map_obs;

	// associated id for single particle observations
	vector<int> associated_id;
	vector<int> map_landmark_index_vector;

	// local variables for association calculation
	double lower_distance;
	int id;
	int map_landmark_index;

	// local variables for weight calculation
	double gauss_norm;
	double exponent;
	double weight;

	for(int i=0;i<num_particles;i++){

		// observations transformed from vehicle to map coordinate system
		for (int j=0;j<observations.size();j++){
			x_map_obs.push_back(particles[i].x + (cos(particles[i].theta)*observations[j].x) - (sin(particles[i].theta)*observations[j].y));
			y_map_obs.push_back(particles[i].y + (sin(particles[i].theta)*observations[j].x) + (cos(particles[i].theta)*observations[j].y));
		}

		// observations associated with corresponding map landmarks
		for (int j=0;j<observations.size();j++){

			// initialize locals
			lower_distance = 9999999999;
			id = 2147483647;
			map_landmark_index = 2147483647;

			// for each map landmark calculate distance to observation. store lower one and corresponding id
			for(int k=0;k<map_landmarks.landmark_list.size();k++){
				double distance = dist(x_map_obs[j], y_map_obs[j], map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
				if(distance<lower_distance){
					lower_distance = distance;
					id = map_landmarks.landmark_list[k].id_i;
					map_landmark_index = k;
				}
			}

			// store id of nearest map landmark
			associated_id.push_back(id);
			map_landmark_index_vector.push_back(map_landmark_index);
		}

		// calculation of particle weight by applying multivariate Gaussian probability density function
		for(int j=0;j<observations.size();j++){

			// gaussian normalization factor
			gauss_norm = (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));

			// exponent
			exponent = (((x_map_obs[j] - map_landmarks.landmark_list[map_landmark_index_vector[j]].x_f)*(x_map_obs[j] - map_landmarks.landmark_list[map_landmark_index_vector[j]].x_f))
					/(2 * std_landmark[0] * std_landmark[0])) +
					(((y_map_obs[j] - map_landmarks.landmark_list[map_landmark_index_vector[j]].y_f)*(y_map_obs[j] - map_landmarks.landmark_list[map_landmark_index_vector[j]].y_f))
					/(2 * std_landmark[1] * std_landmark[1]));

			// weight calculation
			weight = weight * gauss_norm * exp(-exponent);
		}

		// store calculated weight
		particles[i].weight = weight;
		weights[i] = weight;

		// clearing of vectors and variables for next particle
		x_map_obs.clear();
		y_map_obs.clear();
		associated_id.clear();
		map_landmark_index_vector.clear();
		weight = 1;
	}
}

void ParticleFilter::resample() {

	// set of current particles
	std::vector<Particle> particles_;

	// normalize weights
	double sum_weights = std::accumulate(weights.begin(), weights.end(), 0.0);
	for(int i=0;i<weights.size();i++){
		weights[i] = weights[i] / sum_weights;
		particles[i].weight = particles[i].weight / sum_weights;
	}

	// implementation of resampling wheel algorithm
	int index = rand() % num_particles;
	double beta = 0.0;
	double mw = *max_element(weights.begin(), weights.end());

	for(int i=0;i<num_particles;i++){
			beta += ((double) rand() / RAND_MAX) * 2.0 * mw;
			while(beta>weights[index]){
				beta -= weights[index];
				index = (index+1)%num_particles;
			}
			particles_.push_back(particles[index]);
	}

	// set particle vector to result after resampling
	particles = particles_;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
