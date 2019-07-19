/* Include the controller definition */
#include "footbot_foraging.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>

#include "/home/michal/argos3-examples/loop_functions/foraging_loop_functions/foraging_loop_functions.h"

#include <fstream>
#include <ctime>

#include <cstdlib>
#include <random>

#include <iostream>
#include <sstream>

/****************************************/
/****************************************/

std::ofstream file;
int FoundItems = 0;
char buffer [80];
char buffer2 [80];
char filename [80];
std::string data_file;
std::string battery_file;
std::string title = "12 hours long; no discharge per collected item; number of food items: 5; ";
std::string id;
int id_value;

float battery_consume_per_food_item = 0.00;         // 0.06 is equal to ten minutes

const int SwarmSize = 10;

int NumberExploringRobots = SwarmSize;
int NumberChargingRobots;

typedef std::mt19937 MyRNG;  // the Mersenne Twister with a popular choice of parameters
time_t seed_val = time(0);

MyRNG rng;
void initialize()
{
    rng.seed(seed_val);
}

std::uniform_int_distribution<uint32_t> uint_dist100(0,100); // range [0,10]


CFootBotForaging::SFoodData::SFoodData() :
   HasFoodItem(false),
   is_exploring(false),
   stuck(false),
   previous_position(0,0),
   FoodItemIdx(0),
   position_counter(1),
   TotalFoodItems(0) {}


void CFootBotForaging::SFoodData::Reset() {
   HasFoodItem = false;
   is_exploring = false;
   stuck = false;
   previous_position.Set(0,0);
   FoodItemIdx = 0;
   TotalFoodItems = 0;
   position_counter = 1;
}

/****************************************/
/****************************************/

/* Extracting a robot's number from its full ID */
void extractIntegerWords(std::string str) {
    std::stringstream ss;

    /* Storing the whole string into string stream */
    ss << str;

    /* Running loop till the end of the stream */
    std::string temp;
    int found;
    while (!ss.eof()) {

        /* extracting word by word from stream */
        ss >> temp;

        /* Checking the given word is integer or not */
        if (std::stringstream(temp) >> found)

        /* To save from space at the end of string */
        temp = "";
    }
    id_value = found;
}

/****************************************/
/****************************************/

CFootBotForaging::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotForaging::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

CFootBotForaging::SStateData::SStateData() :
   ProbRange(0.0f, 1.0f) {}

void CFootBotForaging::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);
      GetNodeAttribute(t_node, "met_robots_factor", MetRobotsFactor);
      GetNodeAttribute(t_node, "met_continuing_robots", MetContinuingRobots);
      GetNodeAttribute(t_node, "met_returning_robots", MetReturningRobots);
      GetNodeAttribute(t_node, "met_new_continuing_robots", MetNewContinuingRobots);
      GetNodeAttribute(t_node, "met_new_returning_robots", MetNewReturningRobots);
      GetNodeAttribute(t_node, "file_name", filename);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}

void CFootBotForaging::SStateData::Reset() {
   State = STATE_RESTING;
   InNest = true;
   Saved = false;
   /* Initially the robot is resting, and by setting RestingTime to
      MinimumRestingTime we force the robots to make a decision at the
      experiment start. If instead we set RestingTime to zero, we would
      have to wait till RestingTime reaches MinimumRestingTime before
      something happens, which is just a waste of time. */
   TimeSearchingForPlaceInNest = 0;
   TimesChecked = 0;
}

/****************************************/
/****************************************/

CFootBotForaging::CFootBotForaging() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcProximity(NULL),
   m_pcLight(NULL),
   m_pcGround(NULL),
   battery(NULL),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

void CFootBotForaging::Init(TConfigurationNode& t_node) {
   try {
      /*
       * Initialize sensors/actuators
       */
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
      m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
      m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
      m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
      m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );
      battery_sensor    = GetSensor  <CCI_BatterySensor    >("battery" );

      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));

       time_t t = time(0);   // get time now
       struct tm * now = localtime( & t );

       strftime (buffer,80,"experiments_data/%Y-%m-%d/",now);
       strftime (buffer2,80,"experiments_data/%Y-%m-%d/battery_",now);
       data_file = buffer + std::string(filename) + ".txt";
       battery_file = buffer2 + std::string(filename) + ".txt";


//       if(GetId() == "fb 0") {
//           file.open (data_file, std::ios::app);
//           file << "Title: " << title << "number of robots: " << SwarmSize << std::endl;
//           file << "Time: " <<  "Exploring: " << " Charging: ";
//           for(int i = 0; i < SwarmSize; i++){
//               file << " FB_" << i << " ";
//           }
//           file << " Found items: " << std::endl;
//           file.close();
//
//           file.open(battery_file, std::ios::app);
//           file << "Time: " << "Decision time: " << "Time difference: " << "ID: " << "Decision charge: " << "Docking charge: " <<
//           "Charge difference: " << "MetContinuingRobots: " << "MetReturningRobots: " << "MetNewContinuingRobots: " <<
//           "MetNewReturningRobots: " << std::endl;
//           file.close();
//       }

       srand((int)time(0));
       initialize();

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot foraging controller for robot \"" << GetId() << "\"", ex);
   }
   /*
    * Initialize other stuff
    */
   /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
   m_pcRNG = CRandom::CreateRNG("argos");
   Reset();
}

/****************************************/
/****************************************/

void CFootBotForaging::ControlStep() {
   switch(m_sStateData.State) {
      case SStateData::STATE_RESTING: {
         Rest();
         break;
      }
      case SStateData::STATE_EXPLORING: {
         Explore();
         break;
      }
      case SStateData::STATE_RETURN_TO_NEST: {
         ReturnToNest();
         break;
      }
      case SStateData::STATE_CHARGING: {
          Charge();
          break;
      }
      case SStateData::STATE_STOP: {
          Stop();
          break;
      }
      case SStateData::STATE_STUCK: {
          Stuck();
          break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }

   /* Saving data to file */
   if(GetId() == "fb 0") {
       file.open(data_file, std::ios::app);
       file << CSimulator::GetInstance().GetSpace().GetSimulationClock() << " " << NumberExploringRobots << " " << NumberChargingRobots;
       file.close();

       CSpace::TMapPerType &batteries = CSimulator::GetInstance().GetSpace().GetEntitiesByType("battery");

       for (auto &map_element : batteries) {
           CBatteryEquippedEntity &battery = *any_cast<CBatteryEquippedEntity *>(map_element.second);
           file.open(data_file, std::ios::app);
           file << " " << battery.GetAvailableCharge();
           file.close();
       }

       file.open(data_file, std::ios::app);
       file << " " << FoundItems << std::endl;
       file.close();
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::Charge() {
    CCI_BatterySensor::SReading reading = battery_sensor->GetReading();
    Real CurrentTime = CSimulator::GetInstance().GetSpace().GetSimulationClock();

    if(!m_sStateData.Saved) {
//        m_sStateData.ChargingInitialTime = CurrentTime;                                   // variable used in the "instant" charging mode
//        m_sStateData.ChargingInitialValue = 100000 * (1 - reading.AvailableCharge);       // variable used in the "instant" charging mode
        m_sStateData.Saved = true;
        NumberChargingRobots++;
        NumberExploringRobots--;

        id = GetId();
        extractIntegerWords(id);

        file.open(battery_file, std::ios::app);
        file << CurrentTime << " " << m_sStateData.DecisionTime << " " << CurrentTime - m_sStateData.DecisionTime << " " << id_value << " " <<
        m_sStateData.DecisionVolatage << " " << reading.AvailableCharge << " " << m_sStateData.DecisionVolatage - reading.AvailableCharge << " " <<
        m_sStateData.MetContinuingRobots << " " << m_sStateData.MetReturningRobots << " " <<
        m_sStateData.MetNewContinuingRobots << " " << m_sStateData.MetNewReturningRobots << std::endl;
        file.close();
    }

//    /* Charging the robot's battery after specified time nd returning to the exploring mode afterwards */
//    CSpace::TMapPerType& batteries = CSimulator::GetInstance().GetSpace().GetEntitiesByType("battery");
//    if(m_sStateData.ChargingInitialValue < (CurrentTime - m_sStateData.ChargingInitialTime)) {
//        for (auto &map_element : batteries) {
//            CBatteryEquippedEntity &battery = *any_cast<CBatteryEquippedEntity *>(map_element.second);
//            std::string id;
//            id = battery.GetRootEntity().GetId();
//            if (GetId() == battery.GetRootEntity().GetId()) {
//                battery.SetAvailableCharge(1);
//            }
//        }
//
//        m_pcLEDs->SetAllColors(CColor::GREEN);
//        m_sStateData.State = SStateData::STATE_EXPLORING;
//        m_sStateData.TimesChecked = 0;
//        for(int j = 0; j < SwarmSize; j++){
//            m_sStateData.ReceivedData[j][0] = 0;
//            m_sStateData.ReceivedData[j][1] = 0;
//        }
//    }

/* Charging the robot's battery gradually and returning to the exploring mode after specified time */
    CSpace::TMapPerType& batteries = CSimulator::GetInstance().GetSpace().GetEntitiesByType("battery");

    for (auto &map_element : batteries) {
        CBatteryEquippedEntity &battery = *any_cast<CBatteryEquippedEntity *>(map_element.second);
        std::string id;
        id = battery.GetRootEntity().GetId();
        if (GetId() == battery.GetRootEntity().GetId()) {
            battery.SetAvailableCharge(battery.GetAvailableCharge() + 0.000345);
            if(battery.GetAvailableCharge() > 1.0){
                battery.SetAvailableCharge(1.0);
            }
        }
    }

    CCI_BatterySensor::SReading readingg = battery_sensor->GetReading();

    battery_level = readingg.AvailableCharge;

    if(readingg.AvailableCharge >= 0.9999) {
        m_pcLEDs->SetAllColors(CColor::GREEN);
        m_sStateData.State = SStateData::STATE_EXPLORING;
        m_sStateData.TimesChecked = 0;
        m_sFoodData.is_exploring = true;
        for(int j = 0; j < SwarmSize; j++){
//            m_sStateData.ReceivedData[j][0] = 0;
//            m_sStateData.ReceivedData[j][1] = 0;
            m_sStateData.ReceivedData[j][2] = 0;
            m_sStateData.ReceivedData[j][3] = 0;
        }
        m_eChargingResult = CONTINUING_TASK;
        m_pcRABA->SetData(1, m_eChargingResult);
        NumberChargingRobots--;
        NumberExploringRobots++;
//        std::cout << GetId()  << " Battery fully charged" << std::endl;
    }

//    std::cout << GetId()  << " Battery charged level: " << reading.AvailableCharge << std::endl;
}

/****************************************/
/****************************************/

void CFootBotForaging::Stop() {
    /*Stop the wheels... */
    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    /* ... and switch to state 'stop' */
    m_pcLEDs->SetAllColors(CColor::RED);
    return;
}

/****************************************/
/****************************************/

void CFootBotForaging::Stuck() {
    if(!m_sFoodData.stuck) {
        m_pcLEDs->SetAllColors(CColor::GREEN);
        m_sStateData.State = SStateData::STATE_EXPLORING;
        m_sFoodData.is_exploring = true;
    } else {
        m_pcWheels->SetLinearVelocity(-0.2f, -1.0f);
    }
}

/****************************************/
/****************************************/

void CFootBotForaging::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Reset food data */
   m_sFoodData.Reset();
   /* Set LED color */
   m_pcLEDs->SetAllColors(CColor::RED);
   /* Clear up the last exploration result */
   m_eChargingResult = CONTINUING_TASK;
   m_pcRABA->ClearData();
   id = GetId();
   extractIntegerWords(id);
   m_pcRABA->SetData(0, id_value);
   m_pcRABA->SetData(1, m_eChargingResult);
}

/****************************************/
/****************************************/

void CFootBotForaging::UpdateState() {
   /* Reset state flags */
   m_sStateData.InNest = false;
   /* Read stuff from the ground sensor */
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
   /*
    * You can say whether you are in the nest by checking the ground sensor
    * placed close to the wheel motors. It returns a value between 0 and 1.
    * It is 1 when the robot is on a white area, it is 0 when the robot
    * is on a black area and it is around 0.5 when the robot is on a gray
    * area. 
    * The foot-bot has 4 sensors like this, two in the front
    * (corresponding to readings 0 and 1) and two in the back
    * (corresponding to reading 2 and 3).  Here we want the back sensors
    * (readings 2 and 3) to tell us whether we are on gray: if so, the              //Changed in order to completely avoid charging area
    * robot is completely in the nest, otherwise it's outside.
    */
   if((tGroundReads[0].Value > 0.25f &&
      tGroundReads[0].Value < 0.75f ) ||
      (tGroundReads[1].Value > 0.25f &&
      tGroundReads[1].Value < 0.75f)) {
      m_sStateData.InNest = true;
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotForaging::CalculateVectorToLight() {
   /* Get readings from light sensor */
   const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
   /* If the light was perceived, return the vector */
   if(cAccumulator.Length() > 0.0f) {
      return CVector2(1.0f, cAccumulator.Angle());
   }
   /* Otherwise, return zero */
   else {
      return CVector2();
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotForaging::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CFootBotForaging::Rest() {
    m_pcLEDs->SetAllColors(CColor::GREEN);
    m_sStateData.State = SStateData::STATE_EXPLORING;
    m_sFoodData.is_exploring = true;
}

/****************************************/
/****************************************/

void CFootBotForaging::Explore() {
   /* We switch to 'return to nest' in two situations:
    * 1. if we have a food item
    * 2. if we have not found a food item for some time;
    *    in this case, the switch is probabilistic
    */
   bool bReturnToNest(false);
    m_sStateData.Saved = false;
   /*
    * Test the first condition: have we found a food item?
    * NOTE: the food data is updated by the loop functions, so
    * here we just need to read it
    */
   if(m_sFoodData.HasFoodItem) {
       /* Discharging the robot's battery after picking a food item to simulate this task (equal to 10 minutes of operation -> 0.06)*/
       CSpace::TMapPerType &batteries = CSimulator::GetInstance().GetSpace().GetEntitiesByType("battery");
       for (auto &map_element : batteries) {
           CBatteryEquippedEntity &battery = *any_cast<CBatteryEquippedEntity *>(map_element.second);
           std::string id;
           id = battery.GetRootEntity().GetId();
           if (GetId() == battery.GetRootEntity().GetId()) {
               battery.SetAvailableCharge(battery.GetAvailableCharge() - battery_consume_per_food_item);
           }
       }
       FoundItems++;
   }

   /*
    * Social rule: listen to what other people have found and modify
    * probabilities accordingly
    */
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
   for(size_t i = 0; i < tPackets.size(); ++i) {
       switch (tPackets[i].Data[1]) {
           case CONTINUING_TASK: {
               m_sStateData.ReceivedData[tPackets[i].Data[0]][0]++;       //increase number of times of meeting an exploring robot
               m_sStateData.ReceivedData[tPackets[i].Data[0]][1] = 0;     //robot must have recharged so previous counter is irrelevant
               m_sStateData.ReceivedData[tPackets[i].Data[0]][2]++;       //increase number of times of meeting an exploring robot
               m_sStateData.ReceivedData[tPackets[i].Data[0]][3] = 0;     //robot must have recharged so previous counter is irrelevant
               break;
           }
           case NAVIGATING_TO_DOCKING_STATION: {
               m_sStateData.ReceivedData[tPackets[i].Data[0]][0] = 0;
               m_sStateData.ReceivedData[tPackets[i].Data[0]][1]++;       //increase number of times of meeting a robot returing to charging area
               m_sStateData.ReceivedData[tPackets[i].Data[0]][2] = 0;       //increase number of times of meeting an exploring robot
               m_sStateData.ReceivedData[tPackets[i].Data[0]][3]++;     //robot must have recharged so previous counter is irrelevant
               break;
           }
           default: {
               break;
           }
       }
   }

   m_sStateData.MetContinuingRobots = 0;
   m_sStateData.MetReturningRobots = 0;
   m_sStateData.MetNewContinuingRobots = 0;
   m_sStateData.MetNewReturningRobots = 0;

   for(int j = 0; j < SwarmSize; j++){
       if(m_sStateData.ReceivedData[j][0] >= 1){
           m_sStateData.MetContinuingRobots++;
       }
       if(m_sStateData.ReceivedData[j][1] >= 1){
           m_sStateData.MetReturningRobots++;
       }
       if(m_sStateData.ReceivedData[j][2] >= 1){
           m_sStateData.MetNewContinuingRobots++;
       }
       if(m_sStateData.ReceivedData[j][3] >= 1){
           m_sStateData.MetNewReturningRobots++;
       }
   }

    if(m_sStateData.MetNewContinuingRobots > 0 || m_sStateData.MetNewReturningRobots > 0) {
        m_sStateData.MetRobotsFactor = (100 * (m_sStateData.MetNewReturningRobots /
                                               (m_sStateData.MetNewReturningRobots + m_sStateData.MetNewContinuingRobots))) / 2;
    }
    else {
        m_sStateData.MetRobotsFactor = 0;
    }

//    if(m_sStateData.MetContinuingRobots > 0 || m_sStateData.MetReturningRobots > 0) {
//        m_sStateData.MetRobotsFactor = (100 * (m_sStateData.MetReturningRobots /
//                                               (m_sStateData.MetReturningRobots + m_sStateData.MetContinuingRobots))) / 2;
//    }
//    else {
//        m_sStateData.MetRobotsFactor = 0;
//    }

//   std::cout << GetId() << " c: " << m_sStateData.MetContinuingRobots << " r: " << m_sStateData.MetReturningRobots
//   << " new_c: " << m_sStateData.MetNewContinuingRobots << " new_r: " << m_sStateData.MetNewReturningRobots <<
//   " r_factor: " << m_sStateData.MetRobotsFactor << std::endl;

    if(m_sFoodData.stuck){
        m_sStateData.State = SStateData::STATE_STUCK;
        m_pcLEDs->SetAllColors(CColor::ORANGE);
        m_sFoodData.is_exploring = false;
        return;
    }

   CCI_BatterySensor::SReading reading = battery_sensor->GetReading();
    battery_level = reading.AvailableCharge;

//   std::cout << GetId() << " Battery level: " << reading.AvailableCharge << std::endl;
//
   int Probablity = uint_dist100(rng);

   if(reading.AvailableCharge <= 0.9 && m_sStateData.TimesChecked == 0) {
       m_sStateData.TimesChecked = 1;
//       std::cout << GetId() << " continuing: " << m_sStateData.MetContinuingRobots << " returining: " << m_sStateData.MetReturningRobots << " robot_factor: " <<
//                 m_sStateData.MetRobotsFactor << std::endl;
       if (Probablity >= (70 + m_sStateData.MetRobotsFactor)) {
           bReturnToNest = true;
       }
   }
   else if(reading.AvailableCharge <= 0.8 && m_sStateData.TimesChecked == 1){
       m_sStateData.TimesChecked = 2;
//       std::cout << GetId() << " continuing: " << m_sStateData.MetContinuingRobots << " returining: " << m_sStateData.MetReturningRobots << " robot_factor: " <<
//                 m_sStateData.MetRobotsFactor << std::endl;
       if (Probablity >= (60 + m_sStateData.MetRobotsFactor)) {
           bReturnToNest = true;
       }
   }
   else if(reading.AvailableCharge <= 0.7 && m_sStateData.TimesChecked == 2){
       m_sStateData.TimesChecked = 3;
//       std::cout << GetId() << " continuing: " << m_sStateData.MetContinuingRobots << " returining: " << m_sStateData.MetReturningRobots << " robot_factor: " <<
//                 m_sStateData.MetRobotsFactor << std::endl;
       if (Probablity >= (50 + m_sStateData.MetRobotsFactor)) {
           bReturnToNest = true;
       }
   }
   else if(reading.AvailableCharge <= 0.6 && m_sStateData.TimesChecked == 3){
       m_sStateData.TimesChecked = 4;
//       std::cout << GetId() << " continuing: " << m_sStateData.MetContinuingRobots << " returining: " << m_sStateData.MetReturningRobots << " robot_factor: " <<
//                 m_sStateData.MetRobotsFactor << std::endl;
       if (Probablity >= (40 + m_sStateData.MetRobotsFactor)) {
           bReturnToNest = true;
       }
   }
   else if(reading.AvailableCharge <= 0.5 && m_sStateData.TimesChecked == 4){
       m_sStateData.TimesChecked = 5;
//       std::cout << GetId() << " continuing: " << m_sStateData.MetContinuingRobots << " returining: " << m_sStateData.MetReturningRobots << " robot_factor: " <<
//                 m_sStateData.MetRobotsFactor << std::endl;
       if (Probablity >= (30 + m_sStateData.MetRobotsFactor)) {
           bReturnToNest = true;
       }
   }
   else if(reading.AvailableCharge <= 0.4 && m_sStateData.TimesChecked == 5){
       m_sStateData.TimesChecked = 6;
//       std::cout << GetId() << " continuing: " << m_sStateData.MetContinuingRobots << " returining: " << m_sStateData.MetReturningRobots << " robot_factor: " <<
//                 m_sStateData.MetRobotsFactor << std::endl;
       if (Probablity >= (20 + m_sStateData.MetRobotsFactor)) {
           bReturnToNest = true;
       }
   }
   else if(reading.AvailableCharge <= 0.3){
       bReturnToNest = true;
//       std::cout << GetId() << " continuing: " << m_sStateData.MetContinuingRobots << " returining: " << m_sStateData.MetReturningRobots << " robot_factor: " <<
//                 m_sStateData.MetRobotsFactor << std::endl;
   }




   /* So, do we return to the nest now? */
   if(bReturnToNest) {
      /* Yes, we do! */
      m_sStateData.TimeSearchingForPlaceInNest = 0;
      m_pcLEDs->SetAllColors(CColor::BLUE);
      m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
      m_eChargingResult = NAVIGATING_TO_DOCKING_STATION;
      m_sFoodData.is_exploring = false;
      id = GetId();
      extractIntegerWords(id);
      m_pcRABA->SetData(0, id_value);
      m_pcRABA->SetData(1, m_eChargingResult);
      m_sStateData.DecisionTime = CSimulator::GetInstance().GetSpace().GetSimulationClock();
      m_sStateData.DecisionVolatage = reading.AvailableCharge;
   }
   else {
      /* No, perform the actual exploration */
      UpdateState();
      /* Get the diffusion vector to perform obstacle avoidance */
      bool bCollision;
      CVector2 cDiffusion = DiffusionVector(bCollision);

      /*
       * If we are in the nest, we combine antiphototaxis with obstacle
       * avoidance
       * Outside the nest, we just use the diffusion vector
       */
      if(m_sStateData.InNest) {
         /*
          * The vector returned by CalculateVectorToLight() points to
          * the light. Thus, the minus sign is because we want to go away
          * from the light.
          */
         SetWheelSpeedsFromVector(
            m_sWheelTurningParams.MaxSpeed * cDiffusion -
            m_sWheelTurningParams.MaxSpeed * 1.0f * CalculateVectorToLight());
      }
      else {
         /* Use the diffusion vector only */
         SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
      }
   }

//    if(reading.AvailableCharge <= 0.0){
//        m_pcLEDs->SetAllColors(CColor::RED);
//        m_sStateData.State = SStateData::STATE_CHARGING;
//        m_sFoodData.is_exploring = false;
//        NumberExploringRobots--;
//        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
//        std::cout << GetId() << " Battery empty " <<  std::endl;
//    }
}

/****************************************/
/****************************************/

void CFootBotForaging::ReturnToNest() {
   /* As soon as you get to the nest, switch to 'resting' */
   UpdateState();
   /* Are we in the nest? */
   if(m_sStateData.InNest) {
       /* Have we looked for a place long enough? */
       if (m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime) {
           /* Yes, stop the wheels... */
           m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
           /* ... and switch to state 'resting' */
           m_pcLEDs->SetAllColors(CColor::RED);
           m_sStateData.State = SStateData::STATE_CHARGING;
           m_sStateData.TimeSearchingForPlaceInNest = 0;
           return;
       } else {
           /* No, keep looking */
           ++m_sStateData.TimeSearchingForPlaceInNest;
       }
   }
   else {
       /* Still outside the nest */
       m_sStateData.TimeSearchingForPlaceInNest = 0;
//       if (m_sFoodData.HasFoodItem) {
//           /* Discharging the robot's battery after picking a food item to simulate this task (equal to 10 minutes of operation -> 0.06)*/
//           CSpace::TMapPerType &batteries = CSimulator::GetInstance().GetSpace().GetEntitiesByType("battery");
//           for (auto &map_element : batteries) {
//               CBatteryEquippedEntity &battery = *any_cast<CBatteryEquippedEntity *>(map_element.second);
//               std::string id;
//               id = battery.GetRootEntity().GetId();
//               if (GetId() == battery.GetRootEntity().GetId()) {
//                   battery.SetAvailableCharge(battery.GetAvailableCharge() - 0.006);
//               }
//           }
//           FoundItems++;
//       }
   }
   /* Keep going */
   bool bCollision;
   SetWheelSpeedsFromVector(
      m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
      m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotForaging, "footbot_foraging_controller")