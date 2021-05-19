//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/Enforcers/RevngEnforcers.h"

using namespace std;
using namespace AutoEnforcer;

Granularity AutoEnforcer::RootGranularity("Root Granularity");
Granularity
  AutoEnforcer::FunctionsGranularity("Function Granularity", RootGranularity);

Kind AutoEnforcer::Binary("Binary Kind", &RootGranularity);
Kind AutoEnforcer::Root("Root kind", &RootGranularity);
Kind AutoEnforcer::RootIsolated("Root Isolated Kind", Root, &RootGranularity);
Kind AutoEnforcer::Support("Support Kind", &RootGranularity);
Kind AutoEnforcer::Object("Object File Kind", &RootGranularity);
Kind AutoEnforcer::Translated("Translated Kind", &RootGranularity);
Kind AutoEnforcer::Dead("Dead Kind", &RootGranularity);

Kind AutoEnforcer::CFepper("CFepper Kind", &FunctionsGranularity);
Kind AutoEnforcer::Isolated("Isolated Kind", &FunctionsGranularity);
Kind AutoEnforcer::ABIEnforced("ABIEnforced Kind",
                               Isolated,
                               &FunctionsGranularity);
