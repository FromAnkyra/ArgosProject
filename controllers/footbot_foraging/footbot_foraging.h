/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example foraging controller for the foot-bot.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/foraging.argos
 */

#ifndef FOOTBOT_FORAGING_H
#define FOOTBOT_FORAGING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotForaging : public CCI_Controller {

public:

   /*
    * This structure holds data about food collecting by the robots
    */
   struct SFoodData {
      bool HasFoodItem;      // true when the robot is carrying a food item
      bool is_exploring;
      bool stuck;
      CVector2 previous_position;
      size_t FoodItemIdx;    // the index of the current food item in the array of available food items
      size_t TotalFoodItems; // the total number of food items carried by this robot during the experiment
      size_t position_counter;

      SFoodData();
      void Reset();
   };

   /*
    * The following variables are used as parameters for the
    * diffusion algorithm. You can set their value in the <parameters>
    * section of the XML configuration file, under the
    * <controllers><footbot_foraging_controller><parameters><diffusion>
    * section.
    */
   struct SDiffusionParams {
      /*
       * Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;

      /* Constructor */
      SDiffusionParams();

      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode& t_tree);
   };

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_foraging_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The four possible states in which the controller can be */
      enum EState {
         STATE_RESTING = 0,
         STATE_EXPLORING,
         STATE_RETURN_TO_NEST,
         STATE_CHARGING,
         STATE_STOP,
         STATE_STUCK
      } State;

      /* True when the robot is in the nest */
      bool InNest;
      bool Saved;

      /* Value of the robot's battery at the beginning of charging */
      Real ChargingInitialValue;
      /* Time when the robot start charging */
      Real ChargingInitialTime;
      /* */
      Real MetRobotsFactor;
      /* */
      Real MetContinuingRobots;
      /* */
      Real MetReturningRobots;
      /* */
      Real MetNewContinuingRobots;
      /* */
      Real MetNewReturningRobots;
      /* How many times returning to the charging area was considered */
      Real TimesChecked;
      /* Array for data from received messages*/
      Real ReceivedData[50][4];
      /* */
      Real DecisionVolatage;
      /* */
      Real DecisionTime;


      /* Used as a range for uniform number generation */
      CRange<Real> ProbRange;
      /* The increase of ExploreToRestProb due to the food rule */
      Real FoodRuleExploreToRestDeltaProb;
      /* The increase of RestToExploreProb due to the food rule */
      Real FoodRuleRestToExploreDeltaProb;
      /* The increase of ExploreToRestProb due to the collision rule */
      Real CollisionRuleExploreToRestDeltaProb;
      /* The increase of RestToExploreProb due to the social rule */
      Real SocialRuleRestToExploreDeltaProb;
      /* The increase of ExploreToRestProb due to the social rule */
      Real SocialRuleExploreToRestDeltaProb;
      /* The number of exploration steps without finding food after which
         a foot-bot starts thinking about going back to the nest */
      size_t MinimumUnsuccessfulExploreTime;
      /* If the robots switched to resting as soon as it enters the nest,
         there would be overcrowding of robots in the border between the
         nest and the rest of the arena. To overcome this issue, the robot
         spends some time looking for a place in the nest before finally
         settling. The following variable contains the minimum time the
         robot must spend in state 'return to nest' looking for a place in
         the nest before switching to the resting state. */
      size_t MinimumSearchForPlaceInNestTime;
      /* The time spent searching for a place in the nest */
      size_t TimeSearchingForPlaceInNest;

      SStateData();
      void Init(TConfigurationNode& t_node);
      void Reset();
   };

public:

   /* Class constructor. */
   CFootBotForaging();
   /* Class destructor. */
   virtual ~CFootBotForaging() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_foraging_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /*
    * Returns true if the robot is currently exploring.
    */
   inline bool IsExploring() const {
      return m_sStateData.State == SStateData::STATE_EXPLORING;
   }

   /*
    * Returns true if the robot is currently resting.
    */
   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_RESTING;
   }

   /*
    * Returns true if the robot is currently returning to the nest.
    */
   inline bool IsReturningToNest() const {
      return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
   }

   /*
    * Returns the food data
    */
   inline SFoodData& GetFoodData() {
      return m_sFoodData;
   }

   double battery_level;

private:

   /*
    * Updates the state information.
    * In pratice, it sets the SStateData::InNest flag.
    * Future, more complex implementations should add their
    * state update code here.
    */
   void UpdateState();

   /*
    * Calculates the vector to the light. Used to perform
    * phototaxis and antiphototaxis.
    */
   CVector2 CalculateVectorToLight();

   /*
    * Calculates the diffusion vector. If there is a close obstacle,
    * it points away from it; it there is none, it points forwards.
    * The b_collision parameter is used to return true or false whether
    * a collision avoidance just happened or not. It is necessary for the
    * collision rule.
    */
   CVector2 DiffusionVector(bool& b_collision);

   /*
    * Gets a direction vector as input and transforms it into wheel
    * actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   /*
    * Executes the resting state.
    */
   void Rest();

   /*
    * Executes the exploring state.
    */
   void Explore();

   /*
    * Executes the return to nest state.
    */
   void ReturnToNest();

   /*
    * Executes charging of a robot.
    */
   void Charge();

   /*
    * Executes the completely discharge state of of a robot.
    */
   void Stop();
    /*
     * Executes unstucking a robot.
     */
    void Stuck();

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the foot-bot motor ground sensor */
   CCI_FootBotMotorGroundSensor* m_pcGround;

   CCI_BatterySensor* battery_sensor;

   CBatteryEquippedEntity battery;


   /* The random number generator */
   CRandom::CRNG* m_pcRNG;

   /* Used in the social rule to communicate the result of the last
    * exploration attempt */
   enum ELastExplorationResult {
      LAST_EXPLORATION_NONE = 0,    // nothing to report
      LAST_EXPLORATION_SUCCESSFUL,  // the last exploration resulted in a food item found
      LAST_EXPLORATION_UNSUCCESSFUL // no food found in the last exploration
   } m_eLastExplorationResult;

    /* Used in the social rule to communicate the current state of
     * the robot */
    enum EChargingResult {
        CONTINUING_TASK = 0,  // the robot continues task
        NAVIGATING_TO_DOCKING_STATION,  // the robot goes to the charging area
        DEAD
    } m_eChargingResult;

   /* The controller state information */
   SStateData m_sStateData;
   /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;
   /* The food data */
   SFoodData m_sFoodData;

};

#endif
