#include "foraging_qt_user_functions.h"
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CForagingQTUserFunctions::CForagingQTUserFunctions() {
   RegisterUserFunction<CForagingQTUserFunctions,CFootBotEntity>(&CForagingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CForagingQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(c_entity.GetControllableEntity().GetController());
//    DrawText(CVector3(0.0, 0.0, 0.3),   // position
//             c_entity.GetId().c_str()); // text
    DrawText(CVector3(0.0, 0.0, 0.3),   // position
             std::to_string(cController.battery_level).c_str()); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CForagingQTUserFunctions, "foraging_qt_user_functions")
