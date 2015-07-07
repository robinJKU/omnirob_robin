#ifndef CUBIC_TRAJECTORY_GENERATOR
#define CUBIC_TRAJECTORY_GENERATOR

#include <iostream>
#include <vector>
#include <cmath>
#include <stdio.h>

class cubic_trajectory{
public:
	// meta data
	unsigned int nr_of_nodes;
	unsigned int nr_of_edges;

	// trajectory input
	std::vector<double> s_node;
	double v_max;
	double a_max;
	double jerk_max;

	// trajectory params
	std::vector<double> T_edge;
	std::vector<double> t_node;
	std::vector<double> v_node;
	std::vector<double> v_edge;
	std::vector<double> T_rising_falling;
	std::vector<double> T_jerk_rising_falling;
	std::vector<double> a_ramp_rising_falling;

	// others
private:
	unsigned int last_edge_index;

public:
	cubic_trajectory( const std::vector<double> s_node_, double v_max_, double a_max_, double jerk_max_ ):
		nr_of_nodes(s_node_.size()), nr_of_edges(s_node_.size()-1), s_node(s_node_), v_max(v_max_), a_max(a_max_), jerk_max(jerk_max_), T_edge(s_node.size()-1), t_node( s_node_.size()), v_node( s_node_.size()), v_edge( s_node_.size()-1), T_rising_falling( 2*(s_node_.size()-1)), T_jerk_rising_falling( 2*(s_node_.size()-1)), a_ramp_rising_falling( 2*(s_node_.size()-1)), last_edge_index(0)
		{}// constructor

	void determine_parameters( void);
	void evaluate_trajectory( const double time_t, double &position, double &velocity, double &acceleration);
	void evaluate_trajectory( const double time_t, double &position, double &velocity);
	void evaluate_trajectory( const double time_t, double &position);
	double total_length( void);

private:
	void evaluate_jerk_rectangle_profile( const double time_t, const double s_0, const double v_0, const double a_ramp, const double T_jerk, const double T, double &position, double &velocity, double &acceleration );

};// cubic trajectory

class cubic_trajectories{
private:
	bool valid_initialized;
	unsigned int nr_of_edges;
	unsigned int nr_of_trajectories;
	std::vector<cubic_trajectory*> trajectory;

public:
	cubic_trajectories( const std::vector<std::vector<double> > s_node_, const std::vector<double> v_max_, const std::vector<double> a_max_, const std::vector<double> jerk_max_ );
	~cubic_trajectories( void );

	bool is_valid_initialized( void );
	void initialize_trajectories( void);
	void evaluate_trajectories( const double time_t, std::vector<double> &position, std::vector<double> &velocity, std::vector<double> &acceleration );
	void evaluate_trajectories( const double time_t, std::vector<double> &position, std::vector<double> &velocity );
	void evaluate_trajectories( const double time_t, std::vector<double> &position );
	double total_length( void);
	
private:
	void determine_parameters( void );
	void synchronize_parameters( void );
};// cubic trajectories

double pown( const double value, const unsigned int n ){
	if( n==0 ){
		return 1.0;
	}else if( n==1 ){
		return value;
	}else if( n%2==0 ){
		return pown( value,n/2)*pown( value,n/2);
	}
	return pown(value, n-1)*value;

}// pow n

double pow2( const double value ){
	return value*value;
}// pow 2

// ----- sign_1 -------------------------------------------
// sign function which returns 1 for value = 0
double sign_1( const double value ){
	if( value<0 ){
		return -1.0;
	}
	return 1.0;
}// sign 1

// ----- sign_0 -------------------------------------------
// sign function which returns 0 for value = 0
double sign_0( const double value ){
	if( value>0 ){
		return 1.0;
	}else if( value==0 ){
		return 0.0;
	}
	return -1.0;
}// sign 0

/**
 * Determines the cubic_trajectory parameters based on a simplified triangular velocity profile.
 * Must be called before evaluating the trajectory.
 */
void cubic_trajectory::determine_parameters( void){
	double T_min; // minimal edge time
	double T_edge_temp; // adapted edge time

	// look for the min diff which is non zero
	if( 8.0*a_max/jerk_max>2.0*v_max/a_max ){
		T_min = 8.0*a_max/jerk_max;
	}else{
		T_min = 2.0*v_max/a_max;
	}

	for( unsigned int edge_ii=0; edge_ii<nr_of_edges; edge_ii++){
		// estimate edge velocities and edge time based on velocities of a velocity rectangle profile
		if( fabs(s_node[edge_ii+1]-s_node[edge_ii])>1e-3){
			v_edge[edge_ii] = sign_0( s_node[edge_ii+1]-s_node[edge_ii])*v_max;
			T_edge_temp = (s_node[edge_ii+1]-s_node[edge_ii])/v_edge[edge_ii];

			if( T_edge_temp<T_min){
				T_edge_temp = T_min;
				v_edge[edge_ii] = (s_node[edge_ii+1]-s_node[edge_ii])/T_min;
			}

			if( T_edge_temp<T_edge[edge_ii]){
				T_edge_temp = T_edge[edge_ii];
				v_edge[edge_ii] = (s_node[edge_ii+1]-s_node[edge_ii])/T_edge[edge_ii];
			}

		}else{
			T_edge_temp = T_edge[edge_ii];
			v_edge[edge_ii] = 0;
		}
		T_edge[edge_ii] = T_edge_temp;

		// select node velocity for inner nodes (2...node_size-1)
		if( edge_ii>0){
			if( fabs(v_node[edge_ii-1])<1e-5 && fabs(v_edge[edge_ii-1])<1e-5){
				v_node[edge_ii] = 0;
			}else if( fabs(v_edge[edge_ii])<1e-5){
				v_node[edge_ii] = v_edge[edge_ii];
			}else{
				v_node[edge_ii] = (v_edge[edge_ii]+v_edge[edge_ii-1])/2.0;
			}

			// determine velocity for rectangle profile
			if( fabs( T_edge[edge_ii-1])>1e-6){
				v_edge[edge_ii-1] = (s_node[edge_ii]-s_node[edge_ii-1])/(T_edge[edge_ii-1]/2.0) - (v_node[edge_ii]+v_node[edge_ii-1])/2.0;
			}else{
				v_edge[edge_ii-1] = 0.0;
			}
			if( fabs(v_edge[edge_ii-1])>v_max ){
				printf("max vel reached in segment %i\n",edge_ii);
				v_edge[edge_ii-1] = sign_1(v_edge[edge_ii-1])*v_max;
				T_edge[edge_ii-1] = 2.0*(s_node[edge_ii]-s_node[edge_ii-1])/(v_edge[edge_ii-1]+(v_node[edge_ii]+v_node[edge_ii-1])/2.0);
			}
		}
	}// for all edges

	// deteremine velocity for last edge
	if( fabs( T_edge[nr_of_edges-1])>1e-6){
		v_edge[nr_of_edges-1] = (s_node[nr_of_edges]-s_node[nr_of_edges-1])/(T_edge[nr_of_edges-1]/2.0) - (v_node[nr_of_edges]+v_node[nr_of_edges-1])/2.0;
	}else{
		v_edge[nr_of_edges-1] = 0.0;
	}
	if( fabs(v_edge[nr_of_edges-1])>v_max ){
		printf("max vel reached in segment %i\n",nr_of_edges);
		v_edge[nr_of_edges-1] = sign_1(v_edge[nr_of_edges-1])*v_max;
		T_edge[nr_of_edges-1] = 2.0*(s_node[nr_of_edges]-s_node[nr_of_edges-1])/(v_edge[nr_of_edges-1]+(v_node[nr_of_edges]+v_node[nr_of_edges-1])/2.0);
	}

// determine accelerations
	for( unsigned int edge_ii=0; edge_ii<nr_of_edges; edge_ii++){
		T_rising_falling[2*edge_ii] = T_edge[edge_ii]/2.0;
		T_rising_falling[2*edge_ii+1] = T_edge[edge_ii]/2.0;
	}

	double s_0, s_1, v_ii, v_edge_ii, v_iip1, v_0, v_1, old_vel, a_rect, a_ramp, jerk, T_jerk, T_rising, T;

	unsigned int edge_ii = 0;
	while( edge_ii<nr_of_edges){
		v_ii = v_node[edge_ii];
		v_edge_ii = v_edge[edge_ii];
		v_iip1 = v_node[edge_ii+1];

		T_rising = T_rising_falling[2*edge_ii];

		// check rising edge
			v_0 = v_ii;
			v_1 = v_edge_ii;

			a_rect = (v_1-v_0)/T_rising; // Beschleunigungsrampe wird so ausgelegt, dass gleiche Fl�che wie bei Rechteckbeschleunigung auftritt
			jerk = sign_0(a_rect)*jerk_max;

			if( fabs(a_rect)>1e-8){
				T_jerk = T_rising/2.0*(1-sqrt( 1.0 - 4.0*a_rect/(T_rising*jerk)));
			}else{
				T_jerk = 0.0;
			}
			a_ramp = T_jerk*jerk;

			// special casses:
			// - a_ramp zu hoch. Erh�hen von T
			if( (fabs(a_ramp)-a_max)/a_max>1e-4 ){
					s_0 = s_node[ edge_ii];
					s_1 = s_0 + v_0*T_rising+0.5*jerk*pow2(T_rising)*T_jerk-0.5*jerk*T_rising*pow2(T_jerk); // use same value as in the old profile

					a_ramp = sign_1(a_ramp)*a_max;
					T_jerk = a_ramp/jerk;

					T_rising = -v_0/(jerk*T_jerk)+T_jerk/2.0 + sqrt( pow2(v_0/(jerk*T_jerk)-T_jerk/2.0) + 2.0*(s_1-s_0)/(jerk*T_jerk));

					old_vel = v_edge_ii;
					v_edge_ii = v_0+0.5*jerk*pow2(T_jerk)+jerk*T_jerk*(-2.0*T_jerk+T_rising)-0.5*jerk*(pow2(T_rising)-pow2(T_rising-T_jerk))+jerk*T_rising*T_jerk;
					v_edge[edge_ii] = v_edge_ii;

					T_rising_falling[2*edge_ii+1] = T_rising_falling[2*edge_ii+1]*(old_vel + v_iip1)/(v_edge_ii + v_iip1);

					printf("Acceleration too high, increase T_edge - edge %i/1", edge_ii);
			}

			T_jerk_rising_falling[2*edge_ii] = T_jerk;
			T_rising_falling[2*edge_ii] = T_rising;
			a_ramp_rising_falling[2*edge_ii] = a_ramp;

		// check falling edge
			s_0 = s_node[edge_ii]+v_0*T_rising+0.5*jerk*pow2(T_rising)*T_jerk-0.5*jerk*T_rising*pow2(T_jerk);
			T = T_rising_falling[2*edge_ii+1];

			v_0 = v_edge_ii;
			v_1 = v_iip1;

			a_rect = (v_1-v_0)/T; // acceleration ramp is designed such, that the area of both, the rectangular and the ramp profile, is equal
			jerk = sign_0(a_rect)*jerk_max;

			if( fabs(a_rect)>1e-8){
				T_jerk = T/2.0*(1.0-sqrt( 1.0 - 4.0*a_rect/(T*jerk)));
			}else{
				T_jerk = 0.0;
			}
			a_ramp = T_jerk*jerk;

			// special cases:
			// - resulting acceleration too high. Reduce edge duration T_edge
			if( (fabs(a_ramp)-a_max)/a_max>1e-3 ){
				s_1 = s_node[edge_ii+1];

				a_ramp = sign_1(a_ramp)*a_max;
				T_jerk = a_ramp/jerk;
				T = v_1/(jerk*T_jerk)+T_jerk/2.0 + sqrt(pow2(v_1/(jerk*T_jerk)+T_jerk/2) - 2.0*(s_1-s_0)/(jerk*T_jerk));

				// T is choosen such, that the same displacement delta_s is done
				old_vel = v_edge_ii;
				v_edge_ii = v_1 - jerk*T*T_jerk + jerk*pow2(T_jerk);
				v_edge[edge_ii] = v_edge_ii;

				T_rising_falling[2*edge_ii] = T_rising_falling[2*edge_ii]*(v_ii + old_vel)/(v_ii + v_edge_ii);
				T_rising_falling[2*edge_ii+1] = T;

				printf("Acceleration too high - increase T_edge - segment %i/2", edge_ii);
				continue;
			}

			T_jerk_rising_falling[2*edge_ii+1] = T_jerk;
			T_rising_falling[2*edge_ii+1] = T;
			a_ramp_rising_falling[2*edge_ii+1] = a_ramp;

		// update time
		T_edge[edge_ii] = T_rising_falling[2*edge_ii] + T_rising_falling[2*edge_ii+1];
		edge_ii = edge_ii+1;
	}

	t_node[0]=0.0;
	for( unsigned int node_ii=1; node_ii<nr_of_nodes; node_ii++){
		t_node[node_ii] = t_node[node_ii-1] + T_edge[node_ii-1];
	}

}// determine parameters

/**
 * Evaluates a underlying primitive function - the jerk rectangle profile.
 *
 * @param[in] time_t The primitives local time
 * @param[in] s_0 Position offset (position for time_t=0)
 * @param[in] v_0 Velocity offset (velocity for time_t=0)
 * @param[in] a_ramp Maximum amplitude of the acceleration. The value may be positive or negativ.
 * @param[in] T_jerk Time length of the jerk phase. One jerk phase is on the beginning and one on the end of primitive.
 * @param[out] position Evaluated position value for time_t
 * @param[out] velocity Evaluated velocity value for time_t
 * @param[out] acceleration Evaluated acceleration value for time_t
 */
void cubic_trajectory::evaluate_jerk_rectangle_profile( const double time_t, const double s_0, const double v_0, const double a_ramp, const double T_jerk, const double T, double &position, double &velocity, double &acceleration ){
		if( (0.0<=time_t) && (time_t<T_jerk)){
           position = s_0 + v_0*time_t + a_ramp*pown(time_t,3)/(6.0*T_jerk);
           velocity = v_0 + a_ramp*pow2(time_t)/(2.0*T_jerk);
           acceleration = a_ramp*time_t/T_jerk;

		}else if( (T_jerk<=time_t) && (time_t<=T-T_jerk)){
           // no acceleration
           position = s_0 + v_0*T_jerk+ a_ramp*pow2(T_jerk)/6.0;
           position = position + (v_0 + a_ramp*T_jerk/2.0)*(time_t-T_jerk);
           position = position + a_ramp*pow2(time_t-T_jerk)/2.0;

           velocity = v_0 + a_ramp*T_jerk/2.0 + a_ramp*(time_t-T_jerk);
           acceleration = a_ramp;

		}else if( T-T_jerk<time_t){
           position = s_0 + v_0*T_jerk+ a_ramp*pow2(T_jerk)/6.0;
           position = position + (v_0 + a_ramp*T_jerk/2.0)*(T-2.0*T_jerk);
           position = position + a_ramp*pow2(T-2.0*T_jerk)/2.0;
           position = position + (v_0 + a_ramp*T_jerk/2.0 + a_ramp*(T-2.0*T_jerk))*(time_t-(T-T_jerk));
           position = position + a_ramp*pow2(time_t - (T-T_jerk))/2.0;
           position = position - a_ramp*pown((time_t-(T-T_jerk)),3)/(6.0*T_jerk);

           velocity = v_0 + a_ramp*T_jerk/2.0 + a_ramp*(T-2.0*T_jerk) + a_ramp*(time_t - (T-T_jerk)) - a_ramp*pow2(time_t-(T-T_jerk))/(2.0*T_jerk);
           acceleration = a_ramp - a_ramp*(time_t-(T-T_jerk))/T_jerk;

		}
}// evaluate jerk rectangle profile

/**
 * Evaluates the trajectory at the specifyed time.
 *
 * @param[in] time_t Point in time under consideration
 * @param[out] position Evaluated position value for time_t
 * @param[out] velocity Evaluated velocity value for time_t
 * @param[out] acceleration Evaluated acceleration value for time_t
 */
void cubic_trajectory::evaluate_trajectory( const double time_t, double &position, double &velocity, double &acceleration){

	   // check if time is inside trajectory intervall
	   if( time_t<=t_node[0] ){
		   position = s_node[0];
		   velocity = 0.0;
		   acceleration = 0.0;
		   return;
	   }
	   if( t_node[nr_of_nodes-1]<=time_t ){
		   position = s_node[nr_of_nodes-1];
		   velocity = 0.0;
		   acceleration = 0.0;
		   return;
	   }

	   // determine edge index
	   unsigned int edge_index = last_edge_index;
	   unsigned int nr_of_tested_edges=0;
	   while (  (nr_of_tested_edges<nr_of_edges)   &&   (!( (t_node[edge_index]<=time_t) && (time_t<=t_node[edge_index+1]) ))  ){
		   nr_of_tested_edges = nr_of_tested_edges + 1;

		   edge_index = edge_index + 1;
		   if( edge_index>=nr_of_edges ){
			   edge_index -= nr_of_edges;
		   }
	   }
	   if( nr_of_tested_edges>nr_of_edges-1 ){
		   printf("WARNING - edge not found\n");
		   position = s_node[nr_of_nodes-1];
		   velocity = 0.0;
		   acceleration = 0.0;
		   return;
	   }else{
		   last_edge_index = edge_index;

		   double s_0 = s_node[edge_index];

		   double v_0 = v_node[edge_index];
		   double v_01 = v_edge[edge_index];

		   double a_ramp;

		   double T, T_jerk;

		   // determine jerk rectangle profile
		   double local_t = time_t - t_node[edge_index];
		   if( local_t <= T_rising_falling[2*edge_index]){
			   // first edge
			   a_ramp = a_ramp_rising_falling[2*edge_index];
			   T = T_rising_falling[2*edge_index];
			   T_jerk = T_jerk_rising_falling[2*edge_index];

		   }else{
			   // second edge
			   // first of all: determine s0
				   // first edge
				   a_ramp = a_ramp_rising_falling[2*edge_index];
				   T = T_rising_falling[2*edge_index];
				   T_jerk = T_jerk_rising_falling[2*edge_index];
				   evaluate_jerk_rectangle_profile( T, s_0, v_0, a_ramp, T_jerk, T, s_0, velocity, acceleration );

			   // evaluate second edge
			   local_t = local_t - T_rising_falling[2*edge_index];

			   v_0 = v_01;
			   a_ramp = a_ramp_rising_falling[2*edge_index+1];
			   T = T_rising_falling[2*edge_index+1];
			   T_jerk = T_jerk_rising_falling[2*edge_index+1];
		   }

		   evaluate_jerk_rectangle_profile( local_t, s_0, v_0, a_ramp, T_jerk, T, position, velocity, acceleration );

	   }

	}// evaluate trajectory -> pos, vel, acc

/**
 * Evaluates the trajectory at the specifyed time.
 *
 * @param[in] time_t Point in time under consideration
 * @param[out] position Evaluated position value for time_t
 * @param[out] velocity Evaluated velocity value for time_t
 */
void cubic_trajectory::evaluate_trajectory( const double time_t, double &position, double &velocity){
	double acceleration;
	evaluate_trajectory( time_t, position, velocity, acceleration );
}// evaluate trajectory -> pos, vel

/**
 * Evaluates the trajectory at the specifyed time.
 *
 * @param[in] time_t Point in time under consideration
 * @param[out] position Evaluated position value for time_t
 */
void cubic_trajectory::evaluate_trajectory( const double time_t, double &position){
	double velocity, acceleration;
	evaluate_trajectory( time_t, position, velocity, acceleration );
}// evaluate trajectory -> pos

/**
 * Returns the total time duration which the trajectory takes
 * @return Total time duration which the trajectory takes
 */
double cubic_trajectory::total_length( void){
	return t_node.back();
}// total length

/**
 * Constructs a trajectory which is defined by intermediate points and maximal velocities, accelerations and jerks
 *
 * @param[in] s_node_ Intermediate points. The first dimension runs through all trajectories and the secound dimension over all node points in this trajectory.
 * @param[in] v_max_ Maximal velocity. The dimension has to be the same as the trajectory dimension.
 * @param[in] a_max_ Maximal acceleration. The dimension has to be the same as the trajectory dimension.
 * @param[in] j_max_ Maximal jerk. The dimension has to be the same as the trajectory dimension.
 */
cubic_trajectories::cubic_trajectories( const std::vector<std::vector<double> > s_node_, const std::vector<double> v_max_, const std::vector<double> a_max_, const std::vector<double> jerk_max_ )
{
	valid_initialized = false;

	// determine sizes
	nr_of_edges=s_node_[0].size()-1;
	nr_of_trajectories=s_node_.size();

	// check sizes
	if( v_max_.size()!=nr_of_trajectories ){
		printf("initialization error - dimension missmatch (s_node,v_max)");
		return;
	}
	if( a_max_.size()!=nr_of_trajectories ){
		printf("initialization error - dimension missmatch (s_node,a_max)");
		return;
	}
	if( jerk_max_.size()!=nr_of_trajectories ){
		printf("initialization error - dimension missmatch (s_node,jerk_max)");
		return;
	}
	for( unsigned int trajectory_ii=1; trajectory_ii<nr_of_trajectories; trajectory_ii++){
		if( s_node_[trajectory_ii].size()-1!=nr_of_edges ){
			printf("initialization error - dimension missmatch (s_node[0],s_node[%i])",trajectory_ii);
			return;
		}
	}// for all trajectories

	// initialize trajectory
	trajectory.resize(nr_of_trajectories);
	for( unsigned int trajectory_ii=0; trajectory_ii<nr_of_trajectories; trajectory_ii++ ){
		trajectory[trajectory_ii] = new cubic_trajectory( s_node_[trajectory_ii], v_max_[trajectory_ii], a_max_[trajectory_ii], jerk_max_[trajectory_ii]);
	}// for all trajectories

	valid_initialized = true;

}// constructor

/**
 * Destructs a cubic_trajectorie object.
 */
cubic_trajectories::~cubic_trajectories( void ){
	if( trajectory.size()>0 ){
		for( unsigned int trajectory_ii=0; trajectory_ii<nr_of_trajectories; trajectory_ii++ ){
			delete trajectory[trajectory_ii];
		}// for all trajectories
	}
}// destructor

/**
 * Returns true if the trajectories are valid initialized. A valid initialization is done through a valid construction followed by a call of initialize_trajectories.
 * @return true if the trajectories is valid initialized
 */
bool cubic_trajectories::is_valid_initialized( void ){
	return valid_initialized;
}// is valid initialized

/**
 * Determines the parameters of each trajectory
 */
void cubic_trajectories::determine_parameters( void){
	for( unsigned int point_ii=0; point_ii<nr_of_trajectories; point_ii++ ){
		trajectory[point_ii]->determine_parameters();
	}// for all points
}// determine parameters

/**
 * Synchronizes the timeintervalls of the trajectories.
 */
void cubic_trajectories::synchronize_parameters( void){
	std::vector<double> max_edge_time(nr_of_edges);

	for( unsigned int edge_ii=0; edge_ii<nr_of_edges; edge_ii++){
		// determine maximum edge time
		max_edge_time[edge_ii] = 0.0;
		for( unsigned int point_ii=0; point_ii<nr_of_trajectories; point_ii++){
			if( max_edge_time[edge_ii]<trajectory[point_ii]->T_edge[edge_ii] ){
				max_edge_time[edge_ii] = trajectory[point_ii]->T_edge[edge_ii];
			}
		}// for all points
	}// for all edges

	for( unsigned int point_ii=0; point_ii<nr_of_trajectories; point_ii++ ){
		for( unsigned int edge_ii=0; edge_ii<nr_of_edges; edge_ii++){
			trajectory[point_ii]->T_edge[edge_ii] = max_edge_time[edge_ii];
		}// for all edges
	}// for all points

}// synchronize parameters

/**
 * Determine the trajetory parameters and synchronize them.
 */
void cubic_trajectories::initialize_trajectories( void){
	if( valid_initialized ){
		for( unsigned int iter=0; iter<3; iter++ ){
			determine_parameters();
			synchronize_parameters();
		}
	}else{
		printf("could not initialize trajectory -> data object not valid initialized\n");
	}
}// initialize trajectory

/**
 * Evaluates the trajectories at time_t.
 *
 * @param[in] time_t The primitives local time
 * @param[out] position Evaluated position value for time_t
 * @param[out] velocity Evaluated velocity value for time_t
 * @param[out] acceleration Evaluated acceleration value for time_t
 */
void cubic_trajectories::evaluate_trajectories( const double time_t, std::vector<double> &position, std::vector<double> &velocity, std::vector<double> &acceleration){
	if( valid_initialized ){
		// resize input vectors
		if( position.size()!=nr_of_trajectories ){
			position.resize(nr_of_trajectories);
		}
		if( velocity.size()!=nr_of_trajectories ){
			velocity.resize(nr_of_trajectories);
		}
		if( acceleration.size()!=nr_of_trajectories ){
			acceleration.resize(nr_of_trajectories);
		}

		// fill vectors
		double position_ii, velocity_ii, acceleration_ii;
		for( unsigned int trajectory_ii=0; trajectory_ii<nr_of_trajectories; trajectory_ii++ ){
			trajectory[trajectory_ii]->evaluate_trajectory(time_t, position_ii, velocity_ii, acceleration_ii);

			position[trajectory_ii] = position_ii;
			velocity[trajectory_ii] = velocity_ii;
			acceleration[trajectory_ii] = acceleration_ii;
		}// for all trajectories
	}else{
		printf("could not evaluate trajectories -> data object not valid initialized\n");
	}

}// evaluate trajectory -> pos, vel, acc

/**
 * Evaluates the trajectories at time_t.
 *
 * @param[in] time_t The primitives local time
 * @param[out] position Evaluated position value for time_t
 * @param[out] velocity Evaluated velocity value for time_t
 */
void cubic_trajectories::evaluate_trajectories( const double time_t, std::vector<double> &position, std::vector<double> &velocity){
	if( valid_initialized ){
		// resize input vectors
		if( position.size()!=nr_of_trajectories ){
			position.resize(nr_of_trajectories);
		}
		if( velocity.size()!=nr_of_trajectories ){
			velocity.resize(nr_of_trajectories);
		}

		// fill vectors
		double position_ii, velocity_ii;
		for( unsigned int trajectory_ii=0; trajectory_ii<nr_of_trajectories; trajectory_ii++ ){
			trajectory[trajectory_ii]->evaluate_trajectory(time_t, position_ii, velocity_ii);

			position[trajectory_ii] = position_ii;
			velocity[trajectory_ii] = velocity_ii;
		}// for all trajectories
	}else{
		printf("could not evaluate trajectories -> data object not valid initialized\n");
	}

}// evaluate trajectory -> pos, vel

/**
 * Evaluates the trajectories at time_t.
 *
 * @param[in] time_t The primitives local time
 * @param[out] position Evaluated position value for time_t
 */
void cubic_trajectories::evaluate_trajectories( const double time_t, std::vector<double> &position){
	if( valid_initialized ){
		// resize input vectors
		if( position.size()!=nr_of_trajectories ){
			position.resize(nr_of_trajectories);
		}

		// fill vectors
		double position_ii;
		for( unsigned int trajectory_ii=0; trajectory_ii<nr_of_trajectories; trajectory_ii++ ){
			trajectory[trajectory_ii]->evaluate_trajectory(time_t, position_ii);

			position[trajectory_ii] = position_ii;
		}// for all trajectories
	}else{
		printf("could not evaluate trajectories -> data object not valid initialized\n");
	}

}// evaluate trajectory -> pos

/**
 * Returns the total time duration which the trajectories takes. If the trajectories aren't synchronized. The maximum value is returned.
 * @return Total time duration which the trajectories takes. If the trajectory is not properly constructed, the returned value is -1.0.
 */
double cubic_trajectories::total_length( void){
	if( trajectory[0]==NULL ){
		return -1.0;
	}

	double max_length=0.0, temp_length;
	for( unsigned int trajectory_ii=0; trajectory_ii<nr_of_trajectories; trajectory_ii++){
		temp_length = trajectory[trajectory_ii]->total_length();
		if( temp_length>max_length ){
			max_length=temp_length;
		}
	}
	return max_length;
}// total length

#endif
