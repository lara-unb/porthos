#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/Control.h>
#include <ompl/config.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <valarray>
#include <string>
#include <limits>
#include <ompl/geometric/PathSimplifier.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
// maximum linear speed
#define VX_MAX 1
// maximum angular speed
#define THETA_DOT_MAX 1
// Define low boundary for the R^2 part of the SE(2) world we're planning
#define LOW_W_BOUNDARY -2.0
// Define high boundary for the R^2 part of the SE(2) world we're planning
#define HIGH_W_BOUNDARY 2.0

namespace ob = ompl::base;
namespace oc = ompl::control;

// Kinematic car model object definition.  This class does NOT use ODESolver to propagate the system.
class KinematicCarModel : public oc::StatePropagator
{
public:
    KinematicCarModel(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
    {
        space_     = si->getStateSpace();
        timeStep_  = 0.01;
    }

    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const
    {
        EulerIntegration(state, control, duration, result);
    }

protected:
    // Explicit Euler Method for numerical integration.
    void EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
    {
        double t = timeStep_;
        std::valarray<double> dstate;
        space_->copyState(result, start);
        while (t < duration + std::numeric_limits<double>::epsilon())
        {
            ode(result, control, dstate);
            update(result, timeStep_ * dstate);
            t += timeStep_;
        }
        if (t + std::numeric_limits<double>::epsilon() > duration)
        {
            ode(result, control, dstate);
            update(result, (t - duration) * dstate);
        }
    }

    // implement the function describing the robot motion: qdot = f(q, u)
    void ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
    {
        const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
        const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

        dstate.resize(3);
        dstate[0] = u[0] * cos(theta);
        dstate[1] = u[0] * sin(theta);
        dstate[2] = u[1];
    }

    // implement y(n+1) = y(n) + d
    void update(ob::State *state, const std::valarray<double> &dstate) const
    {
        ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
        s.setX(s.getX() + dstate[0]);
        s.setY(s.getY() + dstate[1]);
        s.setYaw(s.getYaw() + dstate[2]);
        space_->enforceBounds(state);
    }

    ob::StateSpacePtr        space_;
    double                   timeStep_;
};
// TODO :  CHANGE POSE FLOATS TO POSE MSG
class PorthosInterface
{
public:
    /*Initializes PorthosInterface object with initial values*/
    PorthosInterface(float x,float y,float theta, std::string vel_topic)
    {
        x_=x;
        y_=y;
        theta_=theta;

        pub_=nh_.advertise<geometry_msgs::Twist>(vel_topic,(uint32_t) 2 ,true);//uncomment to latch
    }
    /*Get x_ value (x coordinate of the base)*/
    float getX(void)
    {
        ROS_DEBUG("Remember to use updatePose() if you want new / up to date values");
        return x_;
    }
    /*Get y_ value (y coordinate of the base)*/
    float getY(void)
    {
        ROS_DEBUG("Remember to use updatePose() if you want new / up to date values");
        return y_;
    }
    /*Get theta_ value (theta coordinate of the base)*/
    float getTheta(void)
    {
        ROS_DEBUG("Remember to use updatePose() if you want new / up to date values");
        return theta_;
    }
    /*set topic name (latched as default)*/
    void setPublisher(std::string vel_topic)
    {
        pub_=nh_.advertise<geometry_msgs::Twist>(vel_topic,(uint32_t) 2 ,true);
    }
    /*set topic name, and gives the user the option to unlatch the topic */
    void setPublisher(std::string vel_topic, bool latch)
    {
        pub_=nh_.advertise<geometry_msgs::Twist>(vel_topic,(uint32_t) 2 ,latch);
    }
    /*Set robot's speed*/
    void setSpeed(float v_x, float theta_dot)
    {

        if(v_x>VX_MAX) v_x=VX_MAX;
        if(v_x<-VX_MAX) v_x=-VX_MAX;
        if(theta_dot>THETA_DOT_MAX) theta_dot=THETA_DOT_MAX;
        if(theta_dot<-THETA_DOT_MAX) theta_dot=-THETA_DOT_MAX;

        geometry_msgs::Twist command;
        command.linear.x=v_x;
        command.angular.z=theta_dot;

        pub_.publish(command);
    }
    /*Tries to listen to tf and get the transformation between base_link and odom_combined*/
    void updatePose(void)
    {
        
        tf::StampedTransform transform;
        try
        {
          listener_.lookupTransform("odom_combined","base_link",ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          ROS_ERROR("POSE VALUES WERE NOT UPDATED!");
          return;
        }

        y_=transform.getOrigin().y();
        x_=transform.getOrigin().x();
        theta_=tf::getYaw(transform.getRotation());
    }
    /*Update the current pose and then print it to the console*/
    void printPose()
    {
        this->updatePose();
        ROS_INFO("x:%f ; y:%f ; theta:%f ",x_,y_,theta_);
    }

protected:
    float x_;
    float y_;
    float theta_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    tf::TransformListener listener_;
};

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    double x=s->getX(), y=s->getY();
    return si->satisfiesBounds(s) && (( (x-1)*(x-1) +y*y )>0.8*0.8);
}

/// @cond IGNORE
class DemoControlSpace : public oc::RealVectorControlSpace
{
public:
    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2) {}
};
/// @endcond
// TO DO: INCLUDE STATE VALIDITY CHECKER INSIDE planning class
class PorthosBasePlan{
public:
    PorthosBasePlan(geometry_msgs::Pose2D start, geometry_msgs::Pose2D goal )
    {
        start_=start;
        goal_=goal;
    }

    std::vector<geometry_msgs::Pose2D>  planWithSimpleSetup(void)
    {
        /// construct the state space we are planning in
        ob::StateSpacePtr space(new ob::SE2StateSpace());

        /// set the bounds for the R^2 part of SE(2)
        ob::RealVectorBounds bounds(2);
        bounds.setLow(LOW_W_BOUNDARY);
        bounds.setHigh(HIGH_W_BOUNDARY);

        space->as<ob::SE2StateSpace>()->setBounds(bounds);

        // create a control space
        oc::ControlSpacePtr cspace(new DemoControlSpace(space));

        // set the bounds for the control space
        ob::RealVectorBounds cbounds(2);
        // set velocity bounds
        cbounds.setLow(0,0); // doesnt allow movement backwards
        cbounds.setHigh(0,VX_MAX);
        // set angle command bounds
        cbounds.setLow(1,-THETA_DOT_MAX);//-0.785
        cbounds.setHigh(1,THETA_DOT_MAX);

        cspace->as<DemoControlSpace>()->setBounds(cbounds);

        // define a simple setup class
        oc::SimpleSetup ss(cspace);

        // set state validity checking for this space
        ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));

        // euler ode solver MUCH FASTER THAN RK4, that is used by default with the ODESolver mehod
        ss.setStatePropagator(oc::StatePropagatorPtr(new KinematicCarModel(ss.getSpaceInformation())));

        ss.getSpaceInformation()->setMinMaxControlDuration(1,30);
        ss.getSpaceInformation()->setPropagationStepSize(0.05);

        // create a start state
        ob::ScopedState<ob::SE2StateSpace> start(space);
        start->setX(start_.x);
        start->setY(start_.y);
        start->setYaw(start_.theta);

        // create a  goal state; use the hard way to set the elements
        ob::ScopedState<ob::SE2StateSpace> goal(space);
        goal->setX(goal_.x);
        goal->setY(goal_.y);
        goal->setYaw(goal_.theta);

        // set the start and goal states
        ss.setStartAndGoalStates(start, goal,0.03);

        // we want to have a reasonable value for the propagation step size
        ss.setup();

        ss.setPlanner(ob::PlannerPtr(new oc::RRT(ss.getSpaceInformation())));
        ob::PlannerStatus solved = ss.solve(30.0);

        if (solved)
        {
            ROS_INFO("Solution found");
            // print the path to file
            std::ofstream file("pathAndControls.txt",std::ostream::out);
            if(ss.getSolutionPath().check()) ROS_INFO("Plano de boa!");
            // ss.getSolutionPath().interpolate();
            ss.getSolutionPath().printAsMatrix(file);
            file.close();
            // get solution path before publishing it to porthos
            int count=ss.getSolutionPath().getControlCount();
            
            std::vector<geometry_msgs::Pose2D> path;
            std::vector<ompl::base::State*>  path_states;
            
            path_states=ss.getSolutionPath().getStates();
            const ob::SE2StateSpace::StateType *se2state;
            
            geometry_msgs::Pose2D aux;
            
            for(int i=1;i<=count;i++)
            {
                se2state=path_states[i]->as<ob::SE2StateSpace::StateType>();

                aux.x=se2state->getX();
                aux.y=se2state->getY();
                aux.theta=se2state->getYaw();

                path.push_back(aux);

            }
            return path;
        }
        else
            ROS_INFO( "No solution found" );
    }

protected:
    geometry_msgs::Pose2D start_;
    geometry_msgs::Pose2D goal_;
};

// TODO: CLASS THAT TAKES THE ROBOT FROM POINT A TO POINT B
//       BY USING A CONTROL LAW FOR SMOOTHER TRAJECTORIES
//       AND BY CORRECTING YAW AS SOON AS THE ROBOT GETS TO THE GOAL POSITION
class PorthosPlanExecution : public PorthosInterface {
// TODO: IMPROVE THE EXECUTION CONSTRAINTS
public:
    PorthosPlanExecution(float x,float y,float theta, std::string vel_topic): PorthosInterface(x,y,theta,vel_topic) {}
    
    void followPath(std::vector<geometry_msgs::Pose2D> path)
    {
        int i=0;
        for(i=0;i<path.size()-1;i++)
        {
            ROS_WARN("EXECUTING TRACK #%d out of %d",i+1, (int)path.size()-1);
            move2Point(path[i+1]);
        }
        correctPose(path[i]);
    }

protected:
    //this function has to calculate distance and use PorthosInterface methods to move Porthos

    void move2Point(geometry_msgs::Pose2D p1 )
    {
        float Kv_x,Ktheta_dot,dist,dist_dir,dir;
        int sign;

        Kv_x=1;
        Ktheta_dot=5;
        dist_dir=1;
        dist=1;
        //todo: move to the point while rotating...
        ROS_INFO("EXECUTION ON PHASE 1");
        dir=atan2(p1.y-y_,p1.x-x_);
        
        while( (dist_dir>0.003 || dist_dir<-0.003) && ros::ok() )
        {
            this->updatePose();
            
            dist_dir=dir-theta_;
            //ROS_INFO("direcao %f ; yaw: %f",dir,theta_);
            dist=sqrt(pow(p1.x-x_,2)+pow(p1.y-y_,2));
            this->setSpeed(0,Ktheta_dot*dist_dir);
        }
        
        dist=1;
        ROS_INFO("EXECUTION ON PHASE 2");
        while( dist>0.03 && ros::ok() )
        {
            this->updatePose();
            dist_dir=dir-theta_;
            dist=sqrt(pow(p1.x-x_,2)+pow(p1.y-y_,2));
            this->setSpeed(Kv_x*dist,Ktheta_dot*dist_dir);
        }
        
        this->setSpeed(0,0);

    }
    void correctPose(geometry_msgs::Pose2D p1)
    {
        float Kv_x,Ktheta_dot,dist,dist_dir,dir;
        int sign;

        Ktheta_dot=3;
        dist_dir=1;
        this->updatePose();
        ROS_INFO("Correcting pose by %f radians",p1.theta-theta_ );
        while( (dist_dir>0.003 || dist_dir<-0.003) && ros::ok() )
        {
            this->updatePose();
            dist_dir=p1.theta-theta_;
            this->setSpeed(0,Ktheta_dot*dist_dir);
        }

        this->setSpeed(0,0);
    }

};

int main(int argc, char **argv)
{

    ros::init(argc,argv,"base_planner");
    ROS_INFO("testing nodes...");
/*
    PorthosInterface p_int(0.0,0.0,0.0,"/cmd_vel");

    p_int.setSpeed(1,1);

    sleep(1);//waits for the robot to actually move before trying to listen to TF
    p_int.updatePose();
    //ROS_INFO("%f",p_int.getX());
    */
/* //Test for PorthosPathExecution class
std::vector<geometry_msgs::Pose2D> test_path;
geometry_msgs::Pose2D aux;
aux.x=0;
aux.y=0;
aux.theta=0;
test_path.push_back(aux);

aux.x=1;
aux.y=1;
aux.theta=0;
test_path.push_back(aux);

aux.x=2;
aux.y=1;
aux.theta=0;
test_path.push_back(aux);

aux.x=2;
aux.y=0;
aux.theta=-M_PI/2;
test_path.push_back(aux);

PorthosPlanExecution exec(0.0,0.0,0.0,"/cmd_vel");
sleep(1);
exec.followPath(test_path);
*/

 //teste da classe de planejamento
    /*
    geometry_msgs::Pose2D start,goal;
    std::vector<geometry_msgs::Pose2D> path;
    start.x=0;
    start.y=0;
    start.theta=3.14;

    goal.x=-1;
    goal.y=5;
    goal.theta=3.14;

    PorthosBasePlan plan(start,goal);
    path=plan.planWithSimpleSetup();

    ROS_INFO("X[1]:%f,Y[1]:%f,YAW[1]:%f",path[1].x,path[1].y,path[1].theta);
    */

    
//teste da classe de interface
    /*
    ros::Rate r(10);
    while(ros::ok())
    {
        p_int.updatePose();
        ROS_INFO("x:%f",p_int.getX());
        ROS_INFO("y:%f",p_int.getY());
        ROS_INFO("theta:%f",p_int.getTheta());
        r.sleep();
        ros::spinOnce();
    }
    */

    //Teste de integracao das classes
    //TODO : IMPROVE EXECUTION CONSTRAINTS AND PATH PLANNING, BUT THE CLASSES INTERATIONS LOOK FINE =]
    geometry_msgs::Pose2D start,goal;
    //std::vector<geometry_msgs::Pose2D> path;
    start.x=0;
    start.y=0;
    start.theta=0;

    goal.x=1.8;
    goal.y=0.5;
    goal.theta=M_PI/2;//M_PI

    PorthosBasePlan plan(start,goal);
    PorthosPlanExecution exec(start.x,start.y,start.theta,"/cmd_vel");
    sleep(1);
    exec.followPath(plan.planWithSimpleSetup());
    ROS_INFO("FINISHED PATH EXECUTION");
    exec.printPose();
    
    ros::spin();
    return 0;
}
