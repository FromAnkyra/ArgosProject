#include "foraging_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <queue>



/****************************************/
/****************************************/

std::string id;
std::queue < int > awaiting_charging_queue;
const int NUmberOfChargingStations = 5;
int charging_list [NUmberOfChargingStations];
int numeric_id = 0;

CForagingLoopFunctions::CForagingLoopFunctions() :
   m_cForagingArenaSideX(-0.9f, 1.7f),                    // ORIGINAL - NOT SCALED
   m_cForagingArenaSideY(-1.7f, 1.7f),
//   m_cForagingArenaSideX(-1.8f, 3.4f),
//   m_cForagingArenaSideY(-3.4f, 3.4f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unCollectedFood(0),
   m_nEnergy(0),
   m_unEnergyPerFoodItem(1),
   m_unEnergyPerWalkingRobot(1) {
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tForaging = GetNode(t_node, "foraging");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
      /* Get the number of food items we want to be scattered from XML */
      UInt32 unFoodItems;
      GetNodeAttribute(tForaging, "items", unFoodItems);
      /* Get the number of food items we want to be scattered from XML */
      GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");
      /* Distribute uniformly the items in the environment */
      for(UInt32 i = 0; i < unFoodItems; ++i) {
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));
      }
      /* Get the output file name from XML */
      GetNodeAttribute(tForaging, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
      /* Get energy gain per item collected */
      GetNodeAttribute(tForaging, "energy_per_item", m_unEnergyPerFoodItem);
      /* Get energy loss per walking robot */
      GetNodeAttribute(tForaging, "energy_per_walking_robot", m_unEnergyPerWalkingRobot);

       for(int j = 0; j < NUmberOfChargingStations; j++){
           charging_list[j] = 1000;
       }

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset() {
   /* Zero the counters */
   m_unCollectedFood = 0;
   m_nEnergy = 0;
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   /* Distribute uniformly the items in the environment */
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
   }
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(c_position_on_plane.GetX() < -1.0f) {                                                     ////ORGINAL - NOT SCALED - WAS -1.0f
      return CColor::GRAY50;
   }
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::PreStep() {
    numeric_id = 0;
    /* Logic to pick and drop food items */
    /*
     * * If a robot is in the nest, drop the food item
     * * If a robot is on a food item, pick it
     * * Each robot can carry only one food item per time
     * */
   UInt32 unWalkingFBs = 0;
   UInt32 unRestingFBs = 0;
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
      /* Count how many foot-bots are in which state */
      if(! cController.IsResting()) ++unWalkingFBs;
      else ++unRestingFBs;
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      /* Get food data */
      CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
      if(sFoodData.position_counter == 1){
          sFoodData.previous_position.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                          cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      }
      /* The foot-bot has a food item */
      if(sFoodData.HasFoodItem) {
          /* Place a new food item on the ground */
          m_cFoodPos[sFoodData.FoodItemIdx].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                                                m_pcRNG->Uniform(m_cForagingArenaSideY));
          /* Drop the food item */
          sFoodData.HasFoodItem = false;
          sFoodData.FoodItemIdx = 0;
          ++sFoodData.TotalFoodItems;
          /* Increase the energy and food count */
          m_nEnergy += m_unEnergyPerFoodItem;
          ++m_unCollectedFood;
          /* The floor texture must be updated */
          m_pcFloor->SetChanged();
//          std::cout << "Food items from loop: " << m_unCollectedFood << std::endl;
      }
      else {
         /* The foot-bot has no food item */
         /* Check whether the foot-bot is out of the nest */
         if(sFoodData.position_counter == 299 && sFoodData.stuck){
             sFoodData.stuck = false;
             sFoodData.position_counter = 0;
         }
         if(sFoodData.position_counter == 99 && !sFoodData.stuck){ //} && sFoodData.is_exploring) {
             if (cPos.GetX() <= (sFoodData.previous_position.GetX() + 0.001) &&
                 cPos.GetX() >= (sFoodData.previous_position.GetX() - 0.001) &&
                 cPos.GetY() <= (sFoodData.previous_position.GetY() + 0.001) &&
                 cPos.GetY() >= (sFoodData.previous_position.GetY() - 0.001)) {
                 sFoodData.stuck = true;
             } else {
                 sFoodData.stuck = false;
             }
             sFoodData.position_counter = 0;
         }
         if(sFoodData.position_counter >= 310){
             sFoodData.position_counter = 0;
         }

//         std::cout << "ID: " << numeric_id << " position: " << cPos.GetX() << " pre_pos: " << sFoodData.previous_position.GetX()
//         <<" position Y: " << cPos.GetY() << " prePos Y: " << sFoodData.previous_position.GetY() << " pos_counter: " << sFoodData.position_counter << std::endl;

         if(cPos.GetX() > -1.0f && sFoodData.is_exploring) {                        ////ORGINAL - NOT SCALED - WAS -1.0f
            /* Check whether the foot-bot is on a food item */
            bool bDone = false;

            for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
               if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
                  /* If so, we move that item out of sight */
                  m_cFoodPos[i].Set(100.0f, 100.f);
                  /* The foot-bot is now carrying an item */
                  sFoodData.HasFoodItem = true;
                  sFoodData.FoodItemIdx = i;
                  /* The floor texture must be updated */
                  m_pcFloor->SetChanged();
                  /* We are done */
                  bDone = true;
               }
            }
         }
         if(cPos.GetX() < -1.0f && sFoodData.wants_to_charge){
             awaiting_charging_queue.push(numeric_id);
             sFoodData.wants_to_charge = false;
             std::cout << "ID " << numeric_id << " enters" << std::endl;
         }
         for (int j = 0; j < NUmberOfChargingStations; j++) {
             if (charging_list[j] == numeric_id) {
                 sFoodData.can_charge = true;
             }
         }
         if(sFoodData.done_charging){
             for (int j = 0; j < NUmberOfChargingStations; j++){
                 if(charging_list[j] == numeric_id){
                     charging_list[j] = 1000;
                 }
             }
             sFoodData.done_charging = false;
             sFoodData.can_charge = false;
         }


      }
      sFoodData.position_counter++;

       numeric_id++;
   }
   /* Update energy expediture due to walking robots */
   m_nEnergy -= unWalkingFBs * m_unEnergyPerWalkingRobot;
   /* Output stuff to file */
   m_cOutput << GetSpace().GetSimulationClock() << "\t"
             << unWalkingFBs << "\t"
             << unRestingFBs << "\t"
             << m_unCollectedFood << "\t"
             << m_nEnergy << std::endl;

   if(!awaiting_charging_queue.empty()) {
       for (int j = 0; j < NUmberOfChargingStations; j++) {
           if (charging_list[j] == 1000 && !awaiting_charging_queue.empty()) {
               charging_list[j] = awaiting_charging_queue.front();
               awaiting_charging_queue.pop();
           }
       }
   }
   std::cout << "Lista: ";
   for (int j = 0; j < NUmberOfChargingStations; j++) {
       std::cout << charging_list[j] << " ";
   }
   std::cout << std::endl;


}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions")
